// world/mesh_file.cpp -- see mesh_file.hpp for the layout and the refusals.
//
// Every field goes through put/get helpers that spell little-endian one byte
// at a time, so the file is the same on every host. Floats travel as their
// IEEE-754 bits (std::bit_cast), so nothing is rounded, and a negative zero
// or a denormal survives.
#include "world/mesh_file.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <ios>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "core/path_text.hpp"

namespace spade {
namespace {

constexpr char kMagic[8] = {'S', 'P', 'A', 'D', 'E', 'M', 'S', 'H'};
// After the header and the source: the hash's 8 bytes.
constexpr uint64_t kTrailerBytes = 8;
static_assert(kMeshFileHeaderBytes == 8 + 4 + 4 + 4 + 4 + 8 + 4, "mesh_file.hpp's table");

// "mesh file '<path>'": every message's subject.
[[nodiscard]] std::string subject(const std::filesystem::path& path) {
    return "mesh file '" + path_text(path) + "'";
}

[[nodiscard]] std::string hex64(uint64_t value) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out = "0x";
    for (int shift = 60; shift >= 0; shift -= 4) {
        out += kHex[(value >> shift) & 0xfu];
    }
    return out;
}

void put32(std::string& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out += static_cast<char>(static_cast<uint8_t>(v >> (8 * i)));
}

void put64(std::string& out, uint64_t v) {
    for (int i = 0; i < 8; ++i) out += static_cast<char>(static_cast<uint8_t>(v >> (8 * i)));
}

[[nodiscard]] uint32_t get32(const unsigned char* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

[[nodiscard]] uint64_t get64(const unsigned char* p) {
    return static_cast<uint64_t>(get32(p)) | (static_cast<uint64_t>(get32(p + 4)) << 32);
}

// The provenance rules both sides share: what the writer refuses to write,
// the reader refuses to read. Empty when it is valid.
[[nodiscard]] std::string provenance_problem(uint64_t source_bytes, uint32_t unit, double tolerance) {
    if (source_bytes > kMaxMeshSourceBytes) {
        return "a provenance source of " + std::to_string(source_bytes) + " bytes, more than the " +
               std::to_string(kMaxMeshSourceBytes) + " a mesh file holds";
    }
    if (unit != static_cast<uint32_t>(MeshUnit::unspecified) && mesh_unit_scale(static_cast<MeshUnit>(unit)) == 0.0) {
        return "provenance unit " + std::to_string(unit) + " is not a MeshUnit";
    }
    if (!std::isfinite(tolerance) || !(tolerance >= 0.0)) {
        return "the provenance tolerance must be finite and >= 0";
    }
    return {};
}

[[nodiscard]] Error refusal(Code code, const std::filesystem::path& path, const std::string& detail) {
    return Error{code, subject(path) + ": " + detail};
}

}  // namespace

Result<void> save_mesh_file(const std::filesystem::path& path, const TriangleMesh& mesh,
                            const MeshProvenance& provenance) {
    const std::string problem = provenance_problem(provenance.source.size(), static_cast<uint32_t>(provenance.unit),
                                                   provenance.tolerance);
    if (!problem.empty()) {
        return std::unexpected(refusal(Code::invalid_argument, path, problem));
    }

    const std::span<const glm::vec3> positions = mesh.positions();
    const std::span<const uint32_t> indices = mesh.indices();
    // Both counts fit a u32: make_mesh() caps the triangles at 2^31, and a
    // closed 2-manifold has fewer vertices than triangles.
    const uint32_t vertex_count = static_cast<uint32_t>(positions.size());
    const uint32_t triangle_count = mesh.triangle_count();

    std::string bytes;
    try {
        bytes.reserve(kMeshFileHeaderBytes + provenance.source.size() + 12 * positions.size() + 4 * indices.size() +
                      kTrailerBytes);
        bytes.append(kMagic, sizeof kMagic);
        put32(bytes, kMeshFileVersion);
        put32(bytes, vertex_count);
        put32(bytes, triangle_count);
        put32(bytes, static_cast<uint32_t>(provenance.unit));
        put64(bytes, std::bit_cast<uint64_t>(provenance.tolerance));
        put32(bytes, static_cast<uint32_t>(provenance.source.size()));
        bytes += provenance.source;
        for (const glm::vec3& p : positions) {
            put32(bytes, std::bit_cast<uint32_t>(p.x));
            put32(bytes, std::bit_cast<uint32_t>(p.y));
            put32(bytes, std::bit_cast<uint32_t>(p.z));
        }
        for (const uint32_t i : indices) put32(bytes, i);
        put64(bytes, mesh.content_hash());

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            return std::unexpected(refusal(Code::io_error, path, "cannot be opened for writing"));
        }
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.close();
        if (!out) {
            return std::unexpected(refusal(Code::io_error, path, "writing it failed"));
        }
    } catch (const std::exception& e) {
        return std::unexpected(refusal(Code::io_error, path, std::string("writing it failed: ") + e.what()));
    }
    return {};
}

Result<LoadedMesh> load_mesh_file(const std::filesystem::path& path, std::optional<uint64_t> expected) {
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(refusal(Code::io_error, path, "cannot be opened"));
        }
        in.seekg(0, std::ios::end);
        const std::streamoff end = in.tellg();
        in.seekg(0, std::ios::beg);
        if (end < 0 || !in) {
            return std::unexpected(refusal(Code::io_error, path, "its size cannot be read"));
        }
        const uint64_t size = static_cast<uint64_t>(end);

        // The fixed header, or as much of it as the file holds.
        unsigned char header[kMeshFileHeaderBytes] = {};
        const std::size_t head = static_cast<std::size_t>(size < kMeshFileHeaderBytes ? size : kMeshFileHeaderBytes);
        in.read(reinterpret_cast<char*>(header), static_cast<std::streamsize>(head));
        if (!in) {
            return std::unexpected(refusal(Code::io_error, path, "reading its header failed"));
        }
        const auto truncated = [&](std::string_view needs) {
            return std::unexpected(refusal(Code::invalid_argument, path,
                                           "truncated: " + std::to_string(size) + " bytes, too short to hold " +
                                               std::string(needs)));
        };

        // The magic: a foreign file is not truncated, even when it is short.
        for (std::size_t i = 0; i < sizeof kMagic && i < head; ++i) {
            if (header[i] != static_cast<unsigned char>(kMagic[i])) {
                return std::unexpected(refusal(Code::invalid_argument, path,
                                               "not a Spade mesh file (it does not begin \"SPADEMSH\")"));
            }
        }
        if (size < 12) return truncated("the magic and the version");
        const uint32_t version = get32(header + 8);
        if (version != kMeshFileVersion) {
            return std::unexpected(refusal(Code::schema_mismatch, path,
                                           "it is version " + std::to_string(version) +
                                               ", and this build reads version " +
                                               std::to_string(kMeshFileVersion) + " only"));
        }
        if (size < kMeshFileHeaderBytes) {
            return truncated("the " + std::to_string(kMeshFileHeaderBytes) + "-byte header");
        }

        const uint32_t vertex_count = get32(header + 12);
        const uint32_t triangle_count = get32(header + 16);
        const uint32_t unit = get32(header + 20);
        const double tolerance = std::bit_cast<double>(get64(header + 24));
        const uint32_t source_bytes = get32(header + 32);

        // Every count is checked before anything is sized by it.
        if (triangle_count > kMaxMeshTriangles) {
            return std::unexpected(refusal(Code::capacity_exceeded, path,
                                           std::to_string(triangle_count) + " triangles, more than the " +
                                               std::to_string(kMaxMeshTriangles) + " a mesh may hold"));
        }
        if (const std::string problem = provenance_problem(source_bytes, unit, tolerance); !problem.empty()) {
            return std::unexpected(refusal(Code::invalid_argument, path, problem));
        }
        // At most 36 + 2^16 + 12 (2^32 - 1) + 12 * 2^31 + 8 < 2^37: no overflow.
        const uint64_t needed = uint64_t{kMeshFileHeaderBytes} + source_bytes + 12 * uint64_t{vertex_count} +
                                12 * uint64_t{triangle_count} + kTrailerBytes;
        if (size != needed) {
            const std::string counts = std::to_string(vertex_count) + " vertices, " + std::to_string(triangle_count) +
                                       " triangles and a " + std::to_string(source_bytes) +
                                       "-byte source need " + std::to_string(needed) + " bytes, and it holds " +
                                       std::to_string(size);
            return std::unexpected(refusal(Code::invalid_argument, path,
                                           size < needed ? "truncated: its " + counts
                                                         : "trailing bytes after the content hash: its " + counts));
        }

        // The rest, now that the file is known to hold exactly it.
        std::vector<unsigned char> body(static_cast<std::size_t>(needed - kMeshFileHeaderBytes));
        in.read(reinterpret_cast<char*>(body.data()), static_cast<std::streamsize>(body.size()));
        if (!in) {
            return std::unexpected(refusal(Code::io_error, path, "reading it failed"));
        }
        const unsigned char* p = body.data();

        LoadedMesh out{};
        out.provenance.source.assign(reinterpret_cast<const char*>(p), source_bytes);
        out.provenance.unit = static_cast<MeshUnit>(unit);
        out.provenance.tolerance = tolerance;
        p += source_bytes;

        // The stored fp32 metres, widened exactly; unit metre scales by 1.0,
        // so make_mesh() stores each float as it was.
        MeshInput input;
        input.unit = MeshUnit::metre;
        input.positions.reserve(vertex_count);
        for (uint32_t v = 0; v < vertex_count; ++v, p += 12) {
            input.positions.emplace_back(static_cast<double>(std::bit_cast<float>(get32(p))),
                                         static_cast<double>(std::bit_cast<float>(get32(p + 4))),
                                         static_cast<double>(std::bit_cast<float>(get32(p + 8))));
        }
        input.indices.reserve(3 * static_cast<std::size_t>(triangle_count));
        for (uint64_t i = 0; i < 3 * uint64_t{triangle_count}; ++i, p += 4) {
            input.indices.push_back(get32(p));
        }
        const uint64_t stored_hash = get64(p);
        body = {};

        Result<std::shared_ptr<const TriangleMesh>> mesh = make_mesh(input, subject(path));
        if (!mesh) {
            return std::unexpected(mesh.error());
        }
        const uint64_t content_hash = (*mesh)->content_hash();
        if (stored_hash != content_hash) {
            return std::unexpected(refusal(Code::invalid_argument, path,
                                           "the stored content hash " + hex64(stored_hash) +
                                               " is not the content's " + hex64(content_hash) +
                                               "; the file is corrupt"));
        }
        if (expected && *expected != content_hash) {
            return std::unexpected(refusal(Code::invalid_argument, path,
                                           "its mesh has content hash " + hex64(content_hash) + ", but " +
                                               hex64(*expected) +
                                               " was named. The mesh changed after it was named: re-name it by "
                                               "the hash it now has, or restore the mesh it was named for"));
        }
        out.mesh = std::move(*mesh);
        return out;
    } catch (const std::exception& e) {
        return std::unexpected(refusal(Code::io_error, path, std::string("reading it failed: ") + e.what()));
    }
}

}  // namespace spade
