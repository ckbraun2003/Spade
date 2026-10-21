// world/mesh_import/stl.cpp -- the binary STL reader. See
// world/mesh_import.hpp for the contract, the refusals and their order.
//
// The 84-byte header is read first, and its count checked against the
// capacity and the file's size before any facet is read. The facets are then
// read in blocks, each vertex's three little-endian float32 widened to
// double, and the soup goes to the shared exact weld and make_mesh().
#include "world/mesh_import.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <expected>
#include <fstream>
#include <ios>
#include <limits>
#include <new>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "core/path_text.hpp"
#include "world/detail/weld.hpp"

namespace spade {
namespace {

constexpr uint64_t kHeaderBytes = 84;  // 80 bytes of header, then the uint32 facet count
constexpr uint64_t kFacetBytes = 50;   // a normal and three vertices (float32 x 12), a 2-byte attribute
constexpr uint64_t kFacetsPerBlock = 4096;

[[nodiscard]] uint32_t le32(const unsigned char* p) noexcept {
    return uint32_t{p[0]} | (uint32_t{p[1]} << 8) | (uint32_t{p[2]} << 16) | (uint32_t{p[3]} << 24);
}

[[nodiscard]] double le_float(const unsigned char* p) noexcept {
    return static_cast<double>(std::bit_cast<float>(le32(p)));
}

[[nodiscard]] std::string num(uint64_t v) {
    return std::to_string(v);
}

[[nodiscard]] Result<std::vector<MeshObject>> read(const std::filesystem::path& path, const std::string& file,
                                                   MeshUnit unit) {
    const auto fail = [&file](Code code, const std::string& detail) {
        return std::unexpected(Error{code, file + ": " + detail});
    };
    std::ifstream in(path, std::ios::binary);
    if (!in) return fail(Code::io_error, "cannot read it");
    in.seekg(0, std::ios::end);
    const std::streamoff end = in.tellg();
    if (!in || end < 0) return fail(Code::io_error, "cannot read it");
    const auto size = static_cast<uint64_t>(end);
    in.seekg(0, std::ios::beg);

    std::array<unsigned char, kHeaderBytes> header{};
    const auto head = static_cast<std::size_t>(std::min(size, kHeaderBytes));
    if (!in.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(head))) {
        return fail(Code::io_error, "cannot read it");
    }
    const bool solid = head >= 5 && std::memcmp(header.data(), "solid", 5) == 0;
    const std::string ascii = "an ASCII STL, which this build does not read (export binary STL)";
    if (size < kHeaderBytes) {
        if (solid) return fail(Code::unavailable, ascii);
        return fail(Code::invalid_argument, num(size) + " bytes, short of the 84-byte header of a binary STL");
    }
    const uint64_t facets = le32(header.data() + 80);
    const uint64_t expected = kHeaderBytes + kFacetBytes * facets;
    if (size != expected && solid) return fail(Code::unavailable, ascii);
    if (facets > kMaxMeshTriangles) {
        return fail(Code::capacity_exceeded, num(facets) + " triangles, more than the " + num(kMaxMeshTriangles) +
                                                 " a mesh may hold");
    }
    if (3 * facets > std::numeric_limits<uint32_t>::max()) {
        return fail(Code::capacity_exceeded, num(3 * facets) + " vertices before welding, more than the " +
                                                 num(std::numeric_limits<uint32_t>::max()) + " an object may index");
    }
    if (size != expected) {
        return fail(Code::invalid_argument, num(size) + " bytes; a binary STL of " + num(facets) + " facets is " +
                                                num(expected) + " bytes");
    }

    std::vector<glm::dvec3> positions;
    positions.reserve(static_cast<std::size_t>(3 * facets));
    std::vector<unsigned char> block;
    for (uint64_t first = 0; first < facets; first += kFacetsPerBlock) {
        const uint64_t count = std::min(kFacetsPerBlock, facets - first);
        block.resize(static_cast<std::size_t>(count * kFacetBytes));
        if (!in.read(reinterpret_cast<char*>(block.data()), static_cast<std::streamsize>(block.size()))) {
            return fail(Code::io_error, "cannot read it from facet " + num(first));
        }
        for (uint64_t f = 0; f < count; ++f) {
            const unsigned char* const facet = block.data() + f * kFacetBytes;
            for (uint64_t v = 0; v < 3; ++v) {
                const unsigned char* const p = facet + 12 * (v + 1);  // past the normal
                const glm::dvec3 q(le_float(p), le_float(p + 4), le_float(p + 8));
                if (!(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z))) {
                    return fail(Code::invalid_argument,
                                "facet " + num(first + f) + ": vertex " + num(v) + " is not finite");
                }
                positions.push_back(q);
            }
        }
    }

    std::vector<uint32_t> indices(positions.size());
    std::iota(indices.begin(), indices.end(), uint32_t{0});
    std::string name = path_text(path.stem());
    const MeshInput input = detail::weld_exact(positions, indices, unit);
    auto built = make_mesh(input, file + ": object '" + name + "'");
    if (!built) return std::unexpected(built.error());
    std::vector<MeshObject> out;
    out.push_back(MeshObject{std::move(name), std::move(*built)});
    return out;
}

}  // namespace

Result<std::vector<MeshObject>> read_stl(const std::filesystem::path& path, MeshUnit unit) {
    const std::string file = path_text(path);
    if (const std::string problem = detail::unit_problem(unit); !problem.empty()) {
        return std::unexpected(Error{Code::invalid_argument, file + ": " + problem});
    }
    try {
        return read(path, file, unit);
    } catch (const std::bad_alloc&) {
        return std::unexpected(Error{Code::capacity_exceeded, file + ": out of memory reading it"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, file + ": unexpected failure: " + e.what()});
    }
}

}  // namespace spade
