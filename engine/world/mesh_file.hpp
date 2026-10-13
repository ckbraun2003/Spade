#pragma once

// ---------------------------------------------------------------------------
// world/mesh_file.hpp -- the .spmesh sidecar: one TriangleMesh on disk
// (CORE-6; the parts and meshes plan, Task B5). A scene file names each mesh
// part's sidecar by a relative path and by the mesh's content hash, as it
// names its world (SCN-001). At run time the engine reads sidecars only; the
// importers (glTF, STL, OBJ, 3MF) and the STEP converter write them.
//
// THE BYTES. Every multi-byte field is little-endian, written and read one
// byte at a time (never a struct, never the host's byte order), with no
// padding anywhere: each field starts where the one before it ends.
//
//   offset            size  field
//   0                 8     magic: the ASCII bytes "SPADEMSH", no terminator
//   8                 4     version: u32, kMeshFileVersion (1)
//   12                4     V: u32, the vertex count
//   16                4     T: u32, the triangle count
//   20                4     unit: u32, MeshProvenance::unit
//   24                8     tolerance: u64, MeshProvenance::tolerance's IEEE-754 binary64 bits
//   32                4     S: u32, the byte length of MeshProvenance::source
//   36                S     source: MeshProvenance::source, UTF-8, no terminator
//   36 + S            12 V  positions: per vertex x, y, z, each a u32 of IEEE-754 binary32 bits:
//                           metres, the part frame, exactly TriangleMesh::positions()
//   36 + S + 12 V     12 T  indices: per triangle three u32, exactly TriangleMesh::indices()
//   36 + S + 12 (V+T) 8     content hash: u64, TriangleMesh::content_hash()
//
// The file is exactly 44 + S + 12 V + 12 T bytes long.
//
// PROVENANCE IS NOT GEOMETRY. The unit, tolerance and source record where a
// mesh came from: the unit the source declared (the positions are metres
// whatever it says; `unspecified` means not recorded), the converter's
// tolerance (CORE-6 D), and the source's name. None of it is in the content
// hash, so re-importing the same geometry from a renamed file names the same
// mesh.
//
// THE MESH IS REBUILT, NEVER TRUSTED. load_mesh_file() hands the stored fp32
// metre positions to make_mesh() unchanged (unit metre, so float(double(x) *
// 1.0) is x, bit for bit), which re-runs every check of the sign's
// precondition and rebuilds the BVH and the pseudonormals. Nothing here
// repairs a mesh (L6): a sidecar make_mesh() refuses is refused with
// make_mesh()'s own message. A repair step belongs in the importers or the
// converter, before a sidecar is written, never here.
//
// NO ALLOCATION FROM AN UNTRUSTED COUNT. The reader reads the 36-byte header,
// checks T against kMaxMeshTriangles and S against kMaxMeshSourceBytes, and
// checks the file's size against 44 + S + 12 V + 12 T (in 64 bits, which
// cannot overflow) before it allocates anything sized by a count.
//
// REFUSALS. Each names the file as "mesh file '<path>'", the path in UTF-8
// (core/path_text.hpp). load_mesh_file(), in this order:
//   io_error          -- the file cannot be opened or read;
//   invalid_argument  -- the first bytes are not "SPADEMSH" (a shorter file
//                        whose bytes begin the magic is truncated, below);
//   invalid_argument  -- truncated: too short to hold the version;
//   schema_mismatch   -- another version ("this build reads version 1");
//   invalid_argument  -- truncated: too short to hold the header;
//   capacity_exceeded -- T past kMaxMeshTriangles;
//   invalid_argument  -- S past kMaxMeshSourceBytes; a unit that is not a
//                        MeshUnit; a tolerance that is not finite and >= 0;
//                        a size other than 44 + S + 12 V + 12 T (truncated,
//                        or trailing bytes);
//   every make_mesh() refusal, with its own code and message;
//   invalid_argument  -- a stored content hash that is not the content's
//                        (the file is corrupt), naming both;
//                        a content hash that is not `expected` (the mesh
//                        changed after it was named), naming both.
// save_mesh_file(): invalid_argument for provenance the reader would refuse;
// io_error when the file cannot be written.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "core/error.hpp"
#include "world/mesh.hpp"

namespace spade {

// The sidecar version this build reads and writes.
inline constexpr uint32_t kMeshFileVersion = 1;

// The fixed header, through S: magic, version, V, T, unit, tolerance, S.
inline constexpr uint32_t kMeshFileHeaderBytes = 36;

// The longest MeshProvenance::source a sidecar holds.
inline constexpr uint32_t kMaxMeshSourceBytes = 65536;

// Where a mesh came from. Stored beside the mesh and outside its content hash.
struct MeshProvenance {
    std::string source{};                    // the source file or object's name, UTF-8; may be empty
    MeshUnit unit = MeshUnit::unspecified;   // the unit the source declared; unspecified: not recorded
    double tolerance = 0.0;                  // m, the converter's tolerance; finite, >= 0; 0: not recorded
    bool operator==(const MeshProvenance&) const = default;
};

// A sidecar's content: the mesh, rebuilt by make_mesh(), and its provenance.
struct LoadedMesh {
    std::shared_ptr<const TriangleMesh> mesh{};
    MeshProvenance provenance{};
};

// Writes `mesh` and `provenance` to `path`, replacing any file there, in the
// layout above. It creates no directory.
[[nodiscard]] Result<void> save_mesh_file(const std::filesystem::path& path, const TriangleMesh& mesh,
                                          const MeshProvenance& provenance = {});

// Reads the sidecar at `path`, and, given `expected`, refuses a mesh whose
// content hash is another (SCN-001's form).
[[nodiscard]] Result<LoadedMesh> load_mesh_file(const std::filesystem::path& path,
                                                std::optional<uint64_t> expected = std::nullopt);

}  // namespace spade
