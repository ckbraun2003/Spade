#pragma once

// ---------------------------------------------------------------------------
// world/mesh_import.hpp -- reading CAD exports into validated meshes (CORE-6;
// the parts and meshes plan, Part B, Tasks B6 and B7). Import only: these readers
// are how geometry gets in. Every object ends in make_mesh(), so a returned
// mesh meets world/mesh.hpp's precondition, and nothing here repairs one.
//
// A SEPARATE TARGET. The readers are spade::mesh_import (STATIC; links
// spade::world, and nlohmann_json privately), so the library the step links
// gains no parser: spade::sim and everything under it never link this
// target (pinned by MeshImport.TheStepNeverLinksTheReaders). The header sits
// in world/ beside mesh.hpp; a consumer that includes it links
// spade::mesh_import.
//
// glTF 2.0 (read_gltf, read_gltf_bytes). A new geometry reader; it shares no
// code with render/gltf.cpp, which is render-only and in spade_render.
//   CONTAINER: .gltf (JSON, with buffers in base64 data: URIs or in files
//     beside it, the URI percent-decoded and read as UTF-8) or .glb (one
//     JSON chunk, then an optional BIN chunk that holds buffer 0). The
//     container is told by the bytes ("glTF" magic), not the extension.
//   UNITS: metres, by the specification (3.6.1), so every MeshInput is
//     MeshUnit::metre.
//   WHAT IS READ: the default scene (`scene`, else scenes[0]), depth first
//     from its root nodes in their listed order, each node's children in
//     theirs (pre-order). Each node with a mesh is one object, so a file is a
//     part list; two nodes naming one mesh are two objects. An object is
//     named by its node, else its mesh, else "node[i]" (i the node index).
//     Every primitive of the mesh is joined into the object, in primitive
//     order. Positions are VEC3 FLOAT; indices are SCALAR UNSIGNED_BYTE,
//     UNSIGNED_SHORT or UNSIGNED_INT, or absent (each three vertices are a
//     triangle). byteStride is honoured. An accessor without a bufferView
//     reads as zeros, as the specification says.
//   TRANSFORMS: each node's local transform (TRS as T * R * S, or a
//     column-major matrix whose last row is 0 0 0 1; not both) is composed
//     down the tree in double, parent * child, and applied to each position
//     in double. A rotation quaternion q is used as q / |q| (the 2 / |q|^2
//     form: + - * / only). A negative determinant reverses the winding, so a
//     mirrored part stays outward.
//   WELDING: exact duplicates only, after the transform: two vertices merge
//     when their double positions are equal by value (so -0 equals +0, and a
//     NaN merges with nothing). Survivors are numbered in first-use order
//     (the order the triangles first name them); a vertex no triangle names
//     is dropped. This turns a glTF's seams (one vertex per face corner, for
//     split normals) back into the shared topology make_mesh() checks. Two
//     different positions are never merged, so two that round to one fp32
//     value are refused by make_mesh() ("weld first"): there is no tolerance.
//   NOT REPAIRED: an object that is not a closed, oriented, manifold solid
//     is refused with make_mesh()'s own message, prefixed
//     "<file>: object '<name>'". There is no heal step.
//
// REFUSALS. Every message starts "<file>: " (the path as UTF-8; for
// read_gltf_bytes, "(glTF bytes)") and names the element: the node, the mesh
// and primitive, the accessor, the bufferView and the buffer, by index.
//   unavailable -- a well-formed file using what this build does not read:
//     any extensionsRequired entry (Draco, meshopt, KHR_mesh_quantization,
//     any other), named; a glTF version other than 2.x, a minVersion other
//     than 2.0, or a .glb container version other than 2; a primitive mode
//     other than TRIANGLES (POINTS, LINES, LINE_LOOP, LINE_STRIP, and also
//     TRIANGLE_STRIP and TRIANGLE_FAN: not converted); a sparse accessor; a
//     POSITION that is not VEC3 FLOAT; morph targets; a skinned node;
//     EXT_mesh_gpu_instancing on a node (reading the base pose or one
//     instance would be silently different geometry); a buffer URI with a
//     scheme other than data:.
//   invalid_argument -- a malformed document: not JSON or not an object; a
//     missing or mistyped field; an index past its array (scene, node,
//     mesh, accessor, bufferView, buffer); a node reached twice; a vertex
//     index out of range; an index or vertex count not a multiple of 3; an
//     accessor past its bufferView, or a bufferView past its buffer; a
//     buffer with no URI (outside a .glb's buffer 0); a byte-length
//     mismatch (a buffer's bytes are not its byteLength; a .glb's BIN chunk
//     may carry up to 3 bytes of padding); a malformed GLB container; an
//     external buffer with no base directory to read it from; a scene with
//     no mesh node; and every make_mesh() refusal.
//   capacity_exceeded -- an object of more than kMaxMeshTriangles
//     triangles, or more than 2^32 - 1 vertices before welding: refused from
//     the accessors' counts, before any data is read. Running out of memory
//     while reading is reported the same way.
//   io_error -- an unreadable file or external buffer.
//   internal -- any other exception, caught (none crosses this API).
//
// THE ORDER. Container, asset, extensionsRequired, the scene, then node by
// node in traversal order: its children, its transform, its mesh, skin and
// instancing. Within an object: each primitive's mode, targets and accessor
// kinds in primitive order; then the object's counts; then each primitive's
// data (ranges, buffers, index values); then the weld and make_mesh(). The
// first refusal is returned.
//
// DETERMINISM. Arrays are read in index order, buffers are read whole and
// decoded byte by byte (little-endian), the transform is + - * / in double
// in a fixed order, and the weld sorts under a total order. The same bytes
// give the same meshes on every platform.
//
// BINARY STL (read_stl). The file has no unit, so the caller declares one:
// MeshUnit::unspecified, or any value that is not a MeshUnit, is refused
// (invalid_argument) before the file is opened.
//   THE FORMAT: an 80-byte header, a little-endian uint32 facet count n, then
//     n facets of 50 bytes: a normal and three vertices, each three
//     little-endian float32, and a 2-byte attribute. The normal and the
//     attribute are not read: the winding says which way a facet faces, and
//     make_mesh() checks it. One object, named by the file's stem.
//   ASCII STL: a file that is not 84 + 50n bytes and starts "solid" is ASCII
//     STL: unavailable ("export binary STL"). The size decides, so a binary
//     file whose header starts "solid", as some exporters write it, is read.
//   WELDING: each facet carries its own three vertices (a soup); the exact
//     weld above joins them into the shared topology make_mesh() checks.
//   THE ORDER: the unit; the file (io_error); a file short of the 84-byte
//     header (unavailable if it starts "solid", else invalid_argument); ASCII;
//     the count against kMaxMeshTriangles and against 2^32 - 1 vertices before
//     welding (capacity_exceeded, before any facet is read); the size (84 + 50n
//     bytes exactly, else invalid_argument); facet by facet, a vertex that is
//     not finite (invalid_argument, naming the facet); then the weld and
//     make_mesh(), its refusals prefixed "<file>: object '<stem>'".
//
// OBJ (read_obj). The unit is declared and refused as for STL. Text, read
// line by line (a line ends at '\n', with an optional '\r' before it);
// fields are separated by spaces and tabs; a line whose first field starts
// with '#' is a comment; a UTF-8 byte order mark before line 1 is skipped.
//   READ: `v x y z`, with up to four more numbers after the three (a weight,
//     or a colour) read and ignored; `f` with three or more corners, each v,
//     v/vt, v/vt/vn or v//vn. Every index is 1-based, a negative one counts
//     back from the latest element of its kind (-1 is the latest), and a
//     corner may name only elements defined above it. vt and vn lines are
//     counted for those range checks and otherwise ignored; s, usemtl and
//     mtllib are ignored.
//   POLYGONS (B7b): a face of more than three corners a, b, c, d, ... is
//     fanned from its first corner into (a, b, c), (a, c, d), ..., keeping its
//     winding, when it is planar and strictly convex:
//     - planar: every corner lies within kObjPlanarTolerance of the face's
//       size (its corners' bounding-box diagonal) of the plane through the
//       corners' mean with Newell's normal;
//     - strictly convex: in the projection along that normal's largest
//       component (the lowest axis on a tie), every corner turns the face's
//       way, and the edges' components along the projection's first axis
//       change sign exactly twice around (so the face winds round once: a
//       star, whose corners all turn one way, does not).
//     Compared in double, + - * only, squared where a length would need a
//     root, so the same text gives the same triangles on every platform.
//   OBJECTS: `o` names objects; in a file with no `o`, `g` does (so a body
//     that `g` splits into material groups stays one object). A name is the
//     rest of its line, trimmed, and must be UTF-8. A bare `o` or `g`, and
//     every face before the first name, belongs to the file's stem. The faces
//     under one name are one object wherever they are in the file; objects
//     come in the order of their first face, and a name with no face is no
//     object. Each object takes the vertices its faces name, welded exactly
//     as above, and its make_mesh() refusals are prefixed
//     "<file>: object '<name>'".
//   NUMBERS: std::from_chars, so no locale applies (a decimal comma is
//     refused); one leading '+' is allowed.
//   REFUSALS, each naming its line ("<file>: line N: ..."); the first in the
//     file is returned.
//     unavailable -- a face of more than three vertices that is not planar
//       (naming its first corner off the plane), is not strictly convex
//       (naming its first corner that does not turn, or saying it winds round
//       more than once), or has no area, each "(triangulate on export)";
//       every other element, named: p, l, free-form geometry (vp,
//       cstype, curv, curv2, surf and the rest) and display attributes (lod
//       and the rest); a line continued by a trailing '\'.
//     invalid_argument -- a v with fewer than three numbers or more than
//       seven, a field that is not a number, a coordinate out of double's
//       range or not finite (the numbers after the three need only be
//       numbers); a face of fewer than three corners, a corner of another
//       form, an index that is not a nonzero integer or names an element not
//       yet defined; a name that is not UTF-8. And, with no line, a file with
//       no face.
//     capacity_exceeded -- more than 2^32 - 1 vertices in the file; and, with
//       no line, an object of more than kMaxMeshTriangles triangles.
//     io_error -- a file that cannot be opened or read (no line; a failed
//       read names the last line read).
//
// For both, running out of memory is capacity_exceeded, and any other
// exception is internal: none crosses this API.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "world/mesh.hpp"

namespace spade {

// How far an OBJ polygon's corner may lie off its plane, as a fraction of the
// face's size (POLYGONS above): 1e-3, so the rounding of a 6-decimal export
// (Blender's) passes on faces down to about a millimetre in a file in metres,
// and the fold a fan makes in a face that passes is a fraction of a degree.
inline constexpr double kObjPlanarTolerance = 1e-3;

// One imported object: its name (UTF-8) and its validated mesh, in metres.
struct MeshObject {
    std::string name;
    std::shared_ptr<const TriangleMesh> mesh;
};

// Reads a .gltf or .glb file. External buffers resolve against its directory.
[[nodiscard]] Result<std::vector<MeshObject>> read_gltf(const std::filesystem::path& path);

// Reads .gltf or .glb bytes already in memory. External buffers resolve
// against `base_dir`; with an empty `base_dir`, one is refused.
[[nodiscard]] Result<std::vector<MeshObject>> read_gltf_bytes(std::span<const std::byte> bytes,
                                                              const std::filesystem::path& base_dir);

// Reads a binary STL file whose lengths are in `unit`: one object.
[[nodiscard]] Result<std::vector<MeshObject>> read_stl(const std::filesystem::path& path, MeshUnit unit);

// Reads an OBJ file whose lengths are in `unit`.
[[nodiscard]] Result<std::vector<MeshObject>> read_obj(const std::filesystem::path& path, MeshUnit unit);

}  // namespace spade
