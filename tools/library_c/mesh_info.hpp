#pragma once

// ---------------------------------------------------------------------------
// library_c/mesh_info.hpp -- the contract's mesh_file_info (L88l, S4): what a mesh file holds, without keeping it. spmesh, glTF, 3MF, binary STL and OBJ.
// It lives in the tool, not in engine/: it reads the CAD readers (spade_mesh_import) and the render glTF loader, which no engine target may link
// (MeshImport.TheStepNeverLinksTheReaders).
//
// Result keys (the contract's section 4): container, triangles, vertices, bbox_min, bbox_max (metres), volume_m3, unit, sha256, solid. A glTF whose objects are not
// all closed solids (a drawing-only presentation mesh) still gives its counts and box, with volume_m3 and solid null. `unit` is the unit the file declares
// ("m", "mm", "cm", "um", "in", "ft"), null where it declares none; a binary STL or OBJ declares none, so the caller names it (`unit` in the request) or the file is refused.
// ---------------------------------------------------------------------------

#include <expected>
#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

#include "scene/library_set.hpp"

namespace spade::libc {

// `unit` is the request's unit text ("mm", ...), or empty; an STL or OBJ needs it. Refused: invalid_request (an unknown unit text, an extension that is no
// container, an STL or OBJ without a unit; field `unit` or `path`), io_error (the file cannot be read), invalid_entry (the file fails its reader; the reader's
// message is the one issue's).
[[nodiscard]] std::expected<nlohmann::json, spade::scene::LibraryRefusal> mesh_file_info(const std::filesystem::path& file, const std::string& unit);

}  // namespace spade::libc
