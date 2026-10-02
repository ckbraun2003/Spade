#pragma once

// ---------------------------------------------------------------------------
// glTF 2.0 mesh loading -- S7a Task R4. A DELIBERATELY SMALL, RENDER-ONLY
// subset: POSITION + indices, per-primitive material grouping (SR-11's
// NON-empty submesh case -- scene.hpp's own comment on MeshData names this
// module as the first PRODUCER of it; render/tessellate.cpp only ever emits
// the empty/implicit case), embedded `data:application/octet-stream;base64,`
// URIs and relative external .bin files.
//
// NORMALS -- READ WHEN PRESENT, GENERATED FLAT ONLY WHEN ABSENT. This is a
// CONDITION, not an unconditional "always generate": task-R4-brief.md states
// it as one twice -- "Normals are generated flat WHEN ABSENT" (its interface
// comment) and, in full, "Generate flat normals WHEN THE FILE HAS NONE --
// tools/gen_meshes.py emits positions+indices only, so this path is the
// common one" (its Step 2). (An earlier revision of this header quoted only
// the clause up to "...embedded ... and external .bin" and stopped short of
// the sentence stating the condition, which read as "NORMAL is never read"
// -- corrected here after review; the fix report on this task records it.)
// A NORMAL attribute, when a primitive carries one, is read through the SAME
// VEC3/FLOAT accessor path already built for POSITION -- its count must
// match POSITION's vertex count -- and its vertices are NOT duplicated: this
// loader appends that primitive's positions/normals/indices UNCHANGED
// (offset into MeshData's running vertex/index space), exactly like the
// file's own shared-vertex accessors. Only when NORMAL is ABSENT does this
// loader fabricate one flat (per-face) normal per triangle, which is what
// forces the vertex duplication MeshData::normals's own doc comment names
// ("flat meshes duplicate vertices", scene.hpp) -- tools/gen_meshes.py, which
// produced every shipped mesh, never emits NORMAL, so today every shipped
// mesh takes the flat-generation path; a hand-authored or licensed asset
// carrying real per-vertex normals takes the other one instead of having
// them silently discarded.
//
// OUT OF SCOPE, DELIBERATELY:
//   - animation, skins, textures, any PBR parameter (this loader never even
//     reads a file's `materials[]` array -- a primitive's raw `material`
//     index is copied into MeshData::submesh_material VERBATIM, unresolved;
//     resolving it against real material data, if that ever happens, is a
//     later task's job, exactly as scene.hpp already delegates "resolving a
//     visual ref to a file" to the caller)
//   - scene-graph node transforms (translation/rotation/scale) -- this loads
//     a MESH's own local geometry, not a scene's node graph
//   - the .glb binary container -- a buffer with no `uri` (the shape an
//     embedded GLB binary chunk takes) is rejected, not silently misread
//   - anything but `mode: 4` (TRIANGLES) primitives, and anything but
//     tightly-packed (no `byteStride`) accessor data
//   - sparse accessors
//   - more than one entry in `meshes[]` -- only meshes[0] is read, matching
//     the one-asset-per-file convention the mesh generators follow
//
// NO ASSET IDS: this module takes a std::filesystem::path. It never learns
// what an asset id such as `mesh:<family>/<name>` is -- resolving an id to a
// path is entirely the CALLER's job (rendering/02-scene-and-appearance.md).
//
// DETERMINISM: loading the same file twice yields byte-identical MeshData.
// Submesh order follows meshes[0].primitives[]'s own array order (arrays
// preserve file order in every JSON representation this module touches) --
// never a map/unordered-container's iteration order.
//
// Errors -- Code::invalid_argument unless noted, one rejection per malformed
// shape rather than a generic "bad file":
//   - the document is not valid JSON, or its top level is not an object
//   - `meshes` (or `accessors`/`bufferViews`/`buffers`) is missing, not an
//     array, or (for `meshes`) empty
//   - meshes[0] has no non-empty `primitives` array
//   - a primitive's `mode` is present and is not 4 (TRIANGLES)
//   - a primitive is missing `attributes.POSITION` or `indices`
//   - an accessor/bufferView/buffer index is out of range
//   - an accessor has a `sparse` override, or its bufferView declares a
//     `byteStride` (interleaved data) -- neither is supported
//   - POSITION's accessor is not VEC3/FLOAT, the indices accessor is not
//     SCALAR/(UNSIGNED_BYTE|UNSIGNED_SHORT|UNSIGNED_INT), or a present
//     NORMAL's accessor is not VEC3/FLOAT -- "unsupported component type"
//   - a present NORMAL accessor's count does not match POSITION's
//   - an accessor's byte range runs past its bufferView, or a bufferView's
//     runs past its buffer
//   - an index references a vertex past the end of POSITION, or an index
//     accessor's count is not a multiple of 3
//   - a position or (present) NORMAL component is not finite, or a
//     GENERATED face normal is not (a degenerate, zero-area triangle --
//     only possible when NORMAL is absent, since only then is one computed)
//   - a buffer has no string `uri` (an embedded GLB binary chunk)
//   - a data URI is not `data:application/octet-stream;base64,...`, or its
//     payload contains an invalid base64 character
//   - a buffer's decoded/read byte length does not match its declared
//     `byteLength`
//   - Code::io_error: `load_gltf` could not open/read the .gltf file, or an
//     external buffer's relative .bin file could not be opened/read
// ---------------------------------------------------------------------------

#include <filesystem>
#include <string_view>

#include "core/error.hpp"
#include "render/scene.hpp"  // MeshData

namespace spade::render {

// Loads a mesh from a .gltf file on disk. `path`'s parent directory is the
// base for resolving any relative external buffer .bin file (see
// parse_gltf's `base_dir`).
[[nodiscard]] Result<MeshData> load_gltf(const std::filesystem::path& path);

// Parses a mesh from already-in-memory glTF JSON text. `base_dir` is where a
// relative buffer `uri` (an external .bin file) is resolved against --
// callers that already have the text (e.g. tests constructing a fixture in
// memory) pass whatever directory their external buffers, if any, live in;
// it is unused when every buffer is an embedded base64 data URI.
[[nodiscard]] Result<MeshData> parse_gltf(std::string_view json_text,
                                           const std::filesystem::path& base_dir);

}  // namespace spade::render
