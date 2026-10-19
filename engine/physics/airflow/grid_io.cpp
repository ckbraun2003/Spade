#include "physics/airflow/grid_io.hpp"

#include <bit>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

namespace spade::physics::airflow {

namespace {

constexpr char kBlobMagic[8] = {'S', 'P', 'A', 'D', 'E', 'G', 'R', 'D'};
constexpr char kFileMagic[8] = {'S', 'P', 'A', 'D', 'E', 'G', 'R', 'F'};
constexpr uint32_t kFileVersion = 1;
constexpr std::size_t kFileHeaderBytes = 8 + 4 + 8;

void put_magic(std::vector<std::byte>& out, const char (&magic)[8]) {
    for (const char c : magic) out.push_back(static_cast<std::byte>(c));
}

[[nodiscard]] bool has_magic(std::span<const std::byte> in, const char (&magic)[8]) noexcept {
    for (std::size_t b = 0; b < 8; ++b) {
        if (in[b] != static_cast<std::byte>(magic[b])) return false;
    }
    return true;
}

void put_u32(std::vector<std::byte>& out, uint32_t v) {
    for (uint32_t b = 0; b < 4; ++b) out.push_back(static_cast<std::byte>((v >> (8u * b)) & 0xffu));
}

void put_u64(std::vector<std::byte>& out, uint64_t v) {
    for (uint32_t b = 0; b < 8; ++b) out.push_back(static_cast<std::byte>((v >> (8u * b)) & 0xffu));
}

[[nodiscard]] uint32_t get_u32(std::span<const std::byte> in, std::size_t at) noexcept {
    uint32_t v = 0;
    for (uint32_t b = 0; b < 4; ++b) v |= static_cast<uint32_t>(std::to_integer<uint8_t>(in[at + b])) << (8u * b);
    return v;
}

[[nodiscard]] uint64_t get_u64(std::span<const std::byte> in, std::size_t at) noexcept {
    uint64_t v = 0;
    for (uint32_t b = 0; b < 8; ++b) v |= static_cast<uint64_t>(std::to_integer<uint8_t>(in[at + b])) << (8u * b);
    return v;
}

void put_floats(std::vector<std::byte>& out, const std::vector<float>& v) {
    if constexpr (std::endian::native == std::endian::little) {
        const std::size_t at = out.size();
        out.resize(at + v.size() * sizeof(float));
        if (!v.empty()) std::memcpy(out.data() + at, v.data(), v.size() * sizeof(float));
    } else {
        for (const float x : v) put_u32(out, std::bit_cast<uint32_t>(x));
    }
}

void get_floats(std::span<const std::byte> in, std::size_t at, std::vector<float>& v) noexcept {
    if constexpr (std::endian::native == std::endian::little) {
        if (!v.empty()) std::memcpy(v.data(), in.data() + at, v.size() * sizeof(float));
    } else {
        for (std::size_t i = 0; i < v.size(); ++i) v[i] = std::bit_cast<float>(get_u32(in, at + 4u * i));
    }
}

[[nodiscard]] std::string shape_text(uint32_t nx, uint32_t ny, uint32_t nz, uint32_t dx_bits) {
    return std::to_string(nx) + "x" + std::to_string(ny) + "x" + std::to_string(nz) + " at dx " +
           std::to_string(std::bit_cast<float>(dx_bits)) + " m";
}

[[nodiscard]] Error fail(Code code, const std::string& why) { return Error{code, "airflow grid: " + why}; }

}  // namespace

std::vector<std::byte> grid_to_bytes(const MacGrid& g) {
    std::vector<std::byte> out;
    out.reserve(kGridBlobHeaderBytes + sizeof(float) * (g.u.size() + g.v.size() + g.w.size() + g.p.size()));
    put_magic(out, kBlobMagic);
    put_u32(out, kGridBlobVersion);
    put_u32(out, g.shape.nx);
    put_u32(out, g.shape.ny);
    put_u32(out, g.shape.nz);
    put_u32(out, std::bit_cast<uint32_t>(g.shape.dx));
    for (const std::vector<float>* a : {&g.u, &g.v, &g.w, &g.p}) put_u64(out, a->size());
    for (const std::vector<float>* a : {&g.u, &g.v, &g.w, &g.p}) put_floats(out, *a);
    return out;
}

Result<void> grid_from_bytes(std::span<const std::byte> bytes, MacGrid& into) {
    if (bytes.size() < kGridBlobHeaderBytes) {
        return std::unexpected(fail(Code::invalid_argument, "the blob is " + std::to_string(bytes.size()) +
                                                                " bytes, shorter than its " +
                                                                std::to_string(kGridBlobHeaderBytes) + "-byte header"));
    }
    if (!has_magic(bytes, kBlobMagic)) {
        return std::unexpected(fail(Code::invalid_argument, "not an airflow grid blob (its magic is not SPADEGRD)"));
    }
    if (const uint32_t version = get_u32(bytes, 8); version != kGridBlobVersion) {
        return std::unexpected(fail(Code::schema_mismatch, "the blob is version " + std::to_string(version) +
                                                               "; this build reads version " +
                                                               std::to_string(kGridBlobVersion)));
    }
    const uint32_t nx = get_u32(bytes, 12);
    const uint32_t ny = get_u32(bytes, 16);
    const uint32_t nz = get_u32(bytes, 20);
    const uint32_t dx_bits = get_u32(bytes, 24);
    const uint32_t into_dx = std::bit_cast<uint32_t>(into.shape.dx);
    if (nx != into.shape.nx || ny != into.shape.ny || nz != into.shape.nz || dx_bits != into_dx) {
        return std::unexpected(fail(Code::schema_mismatch,
                                    "the blob holds a " + shape_text(nx, ny, nz, dx_bits) +
                                        " grid; the grid it restores into is " +
                                        shape_text(into.shape.nx, into.shape.ny, into.shape.nz, into_dx)));
    }
    std::vector<float>* arrays[4] = {&into.u, &into.v, &into.w, &into.p};
    const char* names[4] = {"u", "v", "w", "p"};
    uint64_t values = 0;
    for (std::size_t a = 0; a < 4; ++a) {
        const uint64_t n = get_u64(bytes, 28 + 8 * a);
        if (n != arrays[a]->size()) {
            return std::unexpected(fail(Code::schema_mismatch, std::string("the blob's ") + names[a] + " holds " +
                                                                   std::to_string(n) + " values; the grid's holds " +
                                                                   std::to_string(arrays[a]->size())));
        }
        values += n;
    }
    const uint64_t want = kGridBlobHeaderBytes + sizeof(float) * values;
    if (bytes.size() != want) {
        return std::unexpected(fail(Code::schema_mismatch, "the blob is " + std::to_string(bytes.size()) +
                                                               " bytes; its header gives " + std::to_string(want)));
    }
    std::size_t at = kGridBlobHeaderBytes;
    for (std::vector<float>* a : arrays) {
        get_floats(bytes, at, *a);
        at += sizeof(float) * a->size();
    }
    return {};
}

Result<void> save_grid(const std::filesystem::path& path, const MacGrid& g, uint64_t step) {
    std::vector<std::byte> out;
    put_magic(out, kFileMagic);
    put_u32(out, kFileVersion);
    put_u64(out, step);
    const std::vector<std::byte> blob = grid_to_bytes(g);
    out.insert(out.end(), blob.begin(), blob.end());
    std::filesystem::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
        f.close();
        if (!f) return std::unexpected(fail(Code::io_error, "could not write " + tmp.generic_string()));
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        return std::unexpected(fail(Code::io_error, "could not rename " + tmp.generic_string() + " over " +
                                                        path.generic_string() + ": " + ec.message()));
    }
    return {};
}

Result<uint64_t> load_grid(const std::filesystem::path& path, MacGrid& into) {
    const std::string where = path.generic_string();
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        return std::unexpected(fail(Code::not_found, "no grid file at " + where));
    }
    std::ifstream f(path, std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (f.bad()) return std::unexpected(fail(Code::io_error, "could not read " + where));
    const std::span<const std::byte> bytes(reinterpret_cast<const std::byte*>(raw.data()), raw.size());
    if (bytes.size() < kFileHeaderBytes || !has_magic(bytes, kFileMagic)) {
        return std::unexpected(fail(Code::invalid_argument, where + " is not an airflow grid file"));
    }
    if (const uint32_t version = get_u32(bytes, 8); version != kFileVersion) {
        return std::unexpected(fail(Code::schema_mismatch, where + " is file version " + std::to_string(version) +
                                                               "; this build reads version " +
                                                               std::to_string(kFileVersion)));
    }
    const uint64_t step = get_u64(bytes, 12);
    if (Result<void> ok = grid_from_bytes(bytes.subspan(kFileHeaderBytes), into); !ok) {
        return std::unexpected(Error{ok.error().code, where + ": " + ok.error().context});
    }
    return step;
}

}  // namespace spade::physics::airflow
