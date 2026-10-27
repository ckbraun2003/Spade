#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"

// ---------------------------------------------------------------------------
// world/mesh_file.hpp: the .spmesh sidecar (the parts and meshes plan, Task
// B5; CORE-6, SCN-001's form for the hash).
//
// The layout is spelt here a second time, by hand, byte by byte (TD-4): the
// writer must produce it, and the reader must read it, so neither side can
// drift with the other. Every file lives in its own unique temp folder, and
// every path carries a non-ASCII letter, so each message is checked for the
// path in UTF-8.
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::LoadedMesh;
using spade::MeshInput;
using spade::MeshProvenance;
using spade::MeshUnit;
using spade::TriangleMesh;
using spade::test::TempDir;
using MeshPtr = std::shared_ptr<const TriangleMesh>;

// A tetrahedron with legs a, b, c along +x, +y, +z, counter-clockwise from
// outside.
[[nodiscard]] MeshInput tetra(double a, double b, double c, MeshUnit unit) {
    MeshInput m;
    m.unit = unit;
    m.positions = {{0.0, 0.0, 0.0}, {a, 0.0, 0.0}, {0.0, b, 0.0}, {0.0, 0.0, c}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    return m;
}

// A cube from `lo` with side `side`, vertex v at lo + side * (v & 1, v >> 1 & 1,
// v >> 2 & 1): test_mesh.cpp's fixture.
[[nodiscard]] MeshInput cube(glm::dvec3 lo, double side, MeshUnit unit) {
    static constexpr uint32_t kTriangles[12][3] = {
        {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
        {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
    };
    MeshInput m;
    m.unit = unit;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u),
                                static_cast<double>((v >> 2) & 1u)};
        m.positions.push_back(lo + side * corner);
    }
    for (const auto& t : kTriangles) m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    return m;
}

[[nodiscard]] MeshPtr built(const MeshInput& input) {
    auto m = spade::make_mesh(input, "fixture");
    EXPECT_TRUE(m.has_value()) << m.error().context;
    return m ? *m : nullptr;
}

// A path from UTF-8 text, on every platform (path(std::string) is the ANSI
// code page on Windows).
[[nodiscard]] std::filesystem::path utf8_path(std::string_view text) {
    return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()), text.size()));
}

// "mesh-é.spmesh" in UTF-8: every file below is named with it.
constexpr std::string_view kFileName = "mesh-\xc3\xa9.spmesh";

[[nodiscard]] std::string hex64(uint64_t v) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out = "0x";
    for (int shift = 60; shift >= 0; shift -= 4) out += kHex[(v >> shift) & 0xfu];
    return out;
}

[[nodiscard]] std::vector<uint8_t> read_bytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void write_bytes(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

// --- The layout, spelt by hand (mesh_file.hpp's table) ----------------------

void put32(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

void put64(std::vector<uint8_t>& out, uint64_t v) {
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

struct Spelling {
    uint32_t version = 1;
    uint32_t vertex_count = 0;
    uint32_t triangle_count = 0;
    uint32_t unit = 0;
    double tolerance = 0.0;
    std::string source{};
    std::vector<glm::vec3> positions{};
    std::vector<uint32_t> indices{};
    uint64_t hash = 0;
};

// The bytes of `s`; V and T are written as given, so a test can lie in them.
[[nodiscard]] std::vector<uint8_t> spell(const Spelling& s) {
    std::vector<uint8_t> out = {'S', 'P', 'A', 'D', 'E', 'M', 'S', 'H'};
    put32(out, s.version);
    put32(out, s.vertex_count);
    put32(out, s.triangle_count);
    put32(out, s.unit);
    put64(out, std::bit_cast<uint64_t>(s.tolerance));
    put32(out, static_cast<uint32_t>(s.source.size()));
    out.insert(out.end(), s.source.begin(), s.source.end());
    for (const glm::vec3& p : s.positions) {
        put32(out, std::bit_cast<uint32_t>(p.x));
        put32(out, std::bit_cast<uint32_t>(p.y));
        put32(out, std::bit_cast<uint32_t>(p.z));
    }
    for (const uint32_t i : s.indices) put32(out, i);
    put64(out, s.hash);
    return out;
}

// What a mesh's sidecar must hold, spelt from the mesh's own accessors.
[[nodiscard]] Spelling spelling_of(const TriangleMesh& mesh, const MeshProvenance& provenance) {
    Spelling s;
    s.vertex_count = static_cast<uint32_t>(mesh.positions().size());
    s.triangle_count = mesh.triangle_count();
    s.unit = static_cast<uint32_t>(provenance.unit);
    s.tolerance = provenance.tolerance;
    s.source = provenance.source;
    s.positions.assign(mesh.positions().begin(), mesh.positions().end());
    s.indices.assign(mesh.indices().begin(), mesh.indices().end());
    s.hash = mesh.content_hash();
    return s;
}

// The float positions of `input`, as a sidecar would store them, widened back
// for make_mesh() in metres.
[[nodiscard]] MeshInput stored_input(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices) {
    MeshInput m;
    m.unit = MeshUnit::metre;
    for (const glm::vec3& p : positions) m.positions.emplace_back(p);
    m.indices = indices;
    return m;
}

[[nodiscard]] bool contains(const std::string& haystack, std::string_view needle) {
    return haystack.find(needle) != std::string::npos;
}

template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

// ===========================================================================

TEST(MeshFile, TheBytesAreSpeltAsTheHeaderSays) {
    const TempDir dir("mesh-file-bytes");
    const std::filesystem::path path = dir.path() / utf8_path(kFileName);
    // Millimetres, so the stored floats are float(x * 0.001): metres, not the source's numbers.
    const MeshPtr mesh = built(tetra(12.5, 10.0, 7.5, MeshUnit::millimetre));
    ASSERT_NE(mesh, nullptr);
    const MeshProvenance provenance{.source = "bracket.step", .unit = MeshUnit::millimetre, .tolerance = 1.0e-5};
    ASSERT_TRUE(spade::save_mesh_file(path, *mesh, provenance).has_value());

    const std::vector<uint8_t> bytes = read_bytes(path);
    const std::vector<uint8_t> expected = spell(spelling_of(*mesh, provenance));
    EXPECT_EQ(bytes, expected);
    ASSERT_EQ(bytes.size(), 44u + 12u + 12u * 4u + 12u * 4u) << "44 + S + 12 V + 12 T";

    // A few fields read off by offset, little-endian, against literals.
    static_assert(static_cast<uint32_t>(MeshUnit::millimetre) == 2);
    EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 8), "SPADEMSH");
    EXPECT_EQ((std::vector<uint8_t>(bytes.begin() + 8, bytes.begin() + 24)),
              (std::vector<uint8_t>{1, 0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 2, 0, 0, 0}))
        << "version 1, V 4, T 4, unit millimetre";
    EXPECT_EQ(bytes[32], 12u) << "S, the source's length";
    // Vertex 1's x is float(12.5 * 0.001) = 0.0125f: 0x3C4CCCCD, low byte first.
    const std::size_t v1x = 36 + 12 + 12;
    EXPECT_EQ((std::vector<uint8_t>(bytes.begin() + v1x, bytes.begin() + v1x + 4)),
              (std::vector<uint8_t>{0xCD, 0xCC, 0x4C, 0x3C}));

    // And the reader reads the hand-spelt bytes, not just its own writer's.
    const std::filesystem::path spelt = dir.path() / utf8_path("spelt-\xc3\xa9.spmesh");
    write_bytes(spelt, expected);
    const spade::Result<LoadedMesh> loaded = spade::load_mesh_file(spelt);
    ASSERT_TRUE(loaded.has_value()) << why(loaded);
    EXPECT_EQ(loaded->mesh->content_hash(), mesh->content_hash());
    EXPECT_EQ(loaded->provenance, provenance);
}

TEST(MeshFile, ARoundTripIsBitExactAndKeepsTheHashAndProvenance) {
    const TempDir dir("mesh-file-trip");
    // Floats with every kind of bit: thirds, tenths, and a negative zero.
    MeshInput input = cube({0.0, 0.1, 1.0 / 3.0}, 0.3, MeshUnit::metre);
    input.positions[0].x = -0.0;
    const MeshPtr mesh = built(input);
    ASSERT_NE(mesh, nullptr);
    ASSERT_TRUE(std::signbit(mesh->positions()[0].x));
    const MeshProvenance provenance{
        .source = "Rahmen \xc3\xbc" "ber alles.STEP", .unit = MeshUnit::inch, .tolerance = 2.5e-6};

    const std::filesystem::path first = dir.path() / utf8_path(kFileName);
    ASSERT_TRUE(spade::save_mesh_file(first, *mesh, provenance).has_value());
    const spade::Result<LoadedMesh> loaded = spade::load_mesh_file(first);
    ASSERT_TRUE(loaded.has_value()) << why(loaded);
    const TriangleMesh& back = *loaded->mesh;

    ASSERT_EQ(back.positions().size(), mesh->positions().size());
    EXPECT_EQ(std::memcmp(back.positions().data(), mesh->positions().data(),
                          mesh->positions().size() * sizeof(glm::vec3)),
              0)
        << "every position bit for bit";
    EXPECT_TRUE(std::signbit(back.positions()[0].x)) << "a negative zero is a bit pattern too";
    EXPECT_TRUE(std::equal(back.indices().begin(), back.indices().end(), mesh->indices().begin(),
                           mesh->indices().end()));
    EXPECT_EQ(back.content_hash(), mesh->content_hash());
    EXPECT_EQ(back.shell_count(), mesh->shell_count());
    EXPECT_EQ(loaded->provenance, provenance);
    EXPECT_EQ(spade::mesh_eval(back, {0.5f, 0.2f, 0.4f}), spade::mesh_eval(*mesh, {0.5f, 0.2f, 0.4f}));

    // Save what was loaded: the same bytes.
    const std::filesystem::path second = dir.path() / utf8_path("again-\xc3\xa9.spmesh");
    ASSERT_TRUE(spade::save_mesh_file(second, back, loaded->provenance).has_value());
    EXPECT_EQ(read_bytes(second), read_bytes(first));

    // No provenance is a valid one: an empty source, an unspecified unit, no tolerance.
    const std::filesystem::path bare = dir.path() / utf8_path("bare-\xc3\xa9.spmesh");
    ASSERT_TRUE(spade::save_mesh_file(bare, *mesh).has_value());
    const spade::Result<LoadedMesh> plain = spade::load_mesh_file(bare);
    ASSERT_TRUE(plain.has_value()) << why(plain);
    EXPECT_EQ(plain->provenance, MeshProvenance{});
    EXPECT_EQ(plain->mesh->content_hash(), mesh->content_hash()) << "provenance is outside the hash";
}

TEST(MeshFile, AnotherHashIsRefusedNamingBoth) {
    const TempDir dir("mesh-file-hash");
    const std::filesystem::path path = dir.path() / utf8_path(kFileName);
    const MeshPtr cube_mesh = built(cube({0.0, 0.0, 0.0}, 1.0, MeshUnit::metre));
    ASSERT_NE(cube_mesh, nullptr);
    ASSERT_TRUE(spade::save_mesh_file(path, *cube_mesh).has_value());

    const spade::Result<LoadedMesh> right = spade::load_mesh_file(path, cube_mesh->content_hash());
    ASSERT_TRUE(right.has_value()) << why(right);

    const uint64_t named = cube_mesh->content_hash() ^ 1u;
    const spade::Result<LoadedMesh> wrong = spade::load_mesh_file(path, named);
    ASSERT_FALSE(wrong.has_value());
    EXPECT_EQ(wrong.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(why(wrong), hex64(cube_mesh->content_hash()))) << why(wrong);
    EXPECT_TRUE(contains(why(wrong), hex64(named))) << why(wrong);
    EXPECT_TRUE(contains(why(wrong), kFileName)) << why(wrong);
}

TEST(MeshFile, ATruncatedOrForeignFileIsRefused) {
    const TempDir dir("mesh-file-bad");
    const std::filesystem::path path = dir.path() / utf8_path(kFileName);
    const MeshPtr mesh = built(tetra(0.25, 0.5, 0.125, MeshUnit::metre));
    ASSERT_NE(mesh, nullptr);
    const Spelling good = spelling_of(*mesh, {.source = "t", .unit = MeshUnit::metre, .tolerance = 0.0});
    const std::vector<uint8_t> bytes = spell(good);
    write_bytes(path, bytes);
    ASSERT_TRUE(spade::load_mesh_file(path).has_value());

    const auto refused = [&](const std::vector<uint8_t>& content, Code code, std::string_view needle) {
        write_bytes(path, content);
        const spade::Result<LoadedMesh> r = spade::load_mesh_file(path);
        ASSERT_FALSE(r.has_value()) << "accepted: " << needle;
        EXPECT_EQ(r.error().code, code) << why(r);
        EXPECT_TRUE(contains(why(r), needle)) << why(r);
        EXPECT_TRUE(contains(why(r), kFileName)) << "the path, in UTF-8: " << why(r);
    };

    // Another format altogether.
    refused({'s', 'c', 'e', 'n', 'e', '_', 'v', 'e', 'r', 's', 'i', 'o', 'n', ':', ' ', '1', '\n'},
            Code::invalid_argument, "not a Spade mesh file");
    std::vector<uint8_t> foreign = bytes;
    foreign[7] = 'X';
    refused(foreign, Code::invalid_argument, "not a Spade mesh file");

    // Every prefix of a good file is truncated, the empty file included.
    for (std::size_t cut = 0; cut < bytes.size(); ++cut) {
        SCOPED_TRACE(cut);
        refused(std::vector<uint8_t>(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(cut)),
                Code::invalid_argument, "truncated");
    }
    std::vector<uint8_t> longer = bytes;
    longer.push_back(0);
    refused(longer, Code::invalid_argument, "trailing");

    // Another version is "upgrade", not "fix your file".
    for (const uint32_t version : {0u, 2u}) {
        Spelling other = good;
        other.version = version;
        refused(spell(other), Code::schema_mismatch, "this build reads version 1");
    }

    // Provenance the writer could not have written.
    Spelling unit = good;
    unit.unit = 99;
    refused(spell(unit), Code::invalid_argument, "unit 99");
    for (const double tolerance : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
        Spelling tol = good;
        tol.tolerance = tolerance;
        refused(spell(tol), Code::invalid_argument, "tolerance");
    }

    // A corrupt hash, and a content that no longer matches its stored hash.
    Spelling hash = good;
    hash.hash ^= uint64_t{1} << 40;
    refused(spell(hash), Code::invalid_argument, "stored content hash " + hex64(hash.hash));
    Spelling moved = good;
    moved.positions[1].x = std::nextafter(moved.positions[1].x, 1.0f);
    refused(spell(moved), Code::invalid_argument, "stored content hash " + hex64(good.hash));

    // A file that is not there.
    std::filesystem::remove(path);
    const spade::Result<LoadedMesh> missing = spade::load_mesh_file(path);
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, Code::io_error);
    EXPECT_TRUE(contains(why(missing), kFileName)) << why(missing);
}

TEST(MeshFile, ACountPastTheFileOrTheCapIsRefusedBeforeAnyAllocation) {
    // Each header below claims far more than the file holds. A reader that
    // sized anything by a count before checking it against the file would ask
    // for gigabytes here; the refusals must come back at once.
    const TempDir dir("mesh-file-counts");
    const std::filesystem::path path = dir.path() / utf8_path(kFileName);
    const MeshPtr mesh = built(tetra(0.25, 0.5, 0.125, MeshUnit::metre));
    ASSERT_NE(mesh, nullptr);
    const Spelling good = spelling_of(*mesh, {});

    const auto refused = [&](const Spelling& s, Code code, std::string_view needle) {
        write_bytes(path, spell(s));
        const spade::Result<LoadedMesh> r = spade::load_mesh_file(path);
        ASSERT_FALSE(r.has_value()) << "accepted: " << needle;
        EXPECT_EQ(r.error().code, code) << why(r);
        EXPECT_TRUE(contains(why(r), needle)) << why(r);
        EXPECT_TRUE(contains(why(r), kFileName)) << why(r);
    };

    // Past kMaxMeshTriangles (2^31): make_mesh()'s cap, refused before the size.
    Spelling capped = good;
    capped.triangle_count = static_cast<uint32_t>(spade::kMaxMeshTriangles) + 1u;
    refused(capped, Code::capacity_exceeded, "2147483649 triangles");
    capped.triangle_count = std::numeric_limits<uint32_t>::max();
    refused(capped, Code::capacity_exceeded, "4294967295 triangles");

    // At the cap, and with the most vertices a u32 counts: the file is checked
    // against 44 + S + 12 V + 12 T, in 64 bits, before anything is sized.
    Spelling huge = good;
    huge.triangle_count = static_cast<uint32_t>(spade::kMaxMeshTriangles);
    huge.vertex_count = std::numeric_limits<uint32_t>::max();
    refused(huge, Code::invalid_argument, "truncated");
    refused(huge, Code::invalid_argument, "77309411360 bytes");  // 44 + 12 * (2^32 - 1) + 12 * 2^31

    // A source length past kMaxMeshSourceBytes, before the size check reads it.
    std::vector<uint8_t> bytes = spell(good);
    bytes[32] = 0xFF;
    bytes[33] = 0xFF;
    bytes[34] = 0xFF;
    bytes[35] = 0xFF;
    write_bytes(path, bytes);
    const spade::Result<LoadedMesh> source = spade::load_mesh_file(path);
    ASSERT_FALSE(source.has_value());
    EXPECT_EQ(source.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(why(source), "source")) << why(source);
    EXPECT_TRUE(contains(why(source), "4294967295")) << why(source);
}

TEST(MeshFile, WhatMakeMeshRefusesIsRefusedWithItsOwnMessage) {
    // The sidecar reader repairs nothing (L6): what make_mesh() refuses, a
    // sidecar cannot smuggle in, and the message is make_mesh()'s, after the
    // file's name.
    const TempDir dir("mesh-file-make");
    const std::filesystem::path path = dir.path() / utf8_path(kFileName);
    const MeshInput closed = cube({0.0, 0.0, 0.0}, 1.0, MeshUnit::metre);
    std::vector<glm::vec3> positions;
    for (const glm::dvec3& p : closed.positions) positions.emplace_back(p);

    struct Case {
        const char* what;
        std::vector<glm::vec3> positions;
        std::vector<uint32_t> indices;
        const char* needle;
    };
    std::vector<uint32_t> open = closed.indices;
    open.resize(open.size() - 3);  // the last triangle removed
    std::vector<uint32_t> flipped = closed.indices;
    std::swap(flipped[1], flipped[2]);
    std::vector<glm::vec3> spare = positions;
    spare.emplace_back(5.0f, 5.0f, 5.0f);  // a vertex no triangle uses
    std::vector<uint32_t> out_of_range = closed.indices;
    out_of_range[4] = 8;
    const std::vector<Case> cases = {
        {"an open mesh", positions, open, "open edge"},
        {"a misoriented triangle", positions, flipped, "misoriented edge"},
        {"an unreferenced vertex", spare, closed.indices, "unreferenced"},
        {"an index out of range", positions, out_of_range, "out of range"},
        {"no triangles", positions, {}, "no triangles"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        Spelling s;
        s.vertex_count = static_cast<uint32_t>(c.positions.size());
        s.triangle_count = static_cast<uint32_t>(c.indices.size() / 3);
        s.positions = c.positions;
        s.indices = c.indices;
        write_bytes(path, spell(s));

        const auto direct = spade::make_mesh(stored_input(c.positions, c.indices), "NAME");
        ASSERT_FALSE(direct.has_value());
        const std::string detail = direct.error().context.substr(std::string_view("NAME: ").size());
        EXPECT_TRUE(contains(detail, c.needle)) << detail;

        const spade::Result<LoadedMesh> r = spade::load_mesh_file(path);
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, direct.error().code);
        const std::string& context = r.error().context;
        ASSERT_GE(context.size(), detail.size()) << context;
        EXPECT_EQ(context.substr(context.size() - detail.size()), detail) << "make_mesh()'s own words: " << context;
        EXPECT_TRUE(contains(context, kFileName)) << context;
    }
}

// REPLACED WHOLE (L7b0's review): the bytes go to a temporary renamed over
// the target, which is never written in place, so a write that fails or is
// stopped part-way leaves no part of a sidecar at its name for a reader -- or
// the editor's Save, which keeps a sidecar it finds -- to take for a whole
// one. A target hard-linked to another name shows it: written in place, the
// other name's bytes would change too.
TEST(MeshFile, ASaveReplacesItsTargetWholeAndNeverWritesItInPlace) {
    const TempDir dir("mesh-file-replace");
    const MeshPtr mesh = built(cube({0.0, 0.0, 0.0}, 0.2, MeshUnit::metre));
    ASSERT_NE(mesh, nullptr);
    const std::filesystem::path other = dir.path() / "other.bin";
    {
        std::ofstream out(other, std::ios::binary);
        out << "the other name's bytes";
    }
    const std::filesystem::path target = dir.path() / "linked.spmesh";
    std::error_code ec;
    std::filesystem::create_hard_link(other, target, ec);
    if (ec) {
        GTEST_SKIP() << "no hard links in " << spade::path_text(dir.path()) << ": " << ec.message();
    }

    const spade::Result<void> saved = spade::save_mesh_file(target, *mesh);
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    const std::vector<uint8_t> kept = read_bytes(other);
    EXPECT_EQ(std::string(kept.begin(), kept.end()), "the other name's bytes") << "the target was written in place";
    const spade::Result<LoadedMesh> back = spade::load_mesh_file(target, mesh->content_hash());
    EXPECT_TRUE(back.has_value()) << (back ? "" : back.error().context);
    std::filesystem::path tmp = target;
    tmp += ".tmp-spade";
    EXPECT_FALSE(std::filesystem::exists(tmp)) << "the temporary outlived the save";
}

// A refused save leaves its target as it was and no temporary beside it:
// here the target is a folder, which no file replaces.
TEST(MeshFile, ARefusedSaveLeavesItsTargetAndNoTemporary) {
    const TempDir dir("mesh-file-refused");
    const MeshPtr mesh = built(cube({0.0, 0.0, 0.0}, 0.2, MeshUnit::metre));
    ASSERT_NE(mesh, nullptr);
    const std::filesystem::path target = dir.path() / "taken.spmesh";
    std::filesystem::create_directories(target / "inside");

    const spade::Result<void> saved = spade::save_mesh_file(target, *mesh);
    ASSERT_FALSE(saved.has_value());
    EXPECT_EQ(saved.error().code, Code::io_error);
    EXPECT_TRUE(std::filesystem::is_directory(target / "inside"));
    std::filesystem::path tmp = target;
    tmp += ".tmp-spade";
    EXPECT_FALSE(std::filesystem::exists(tmp)) << "the temporary outlived the refusal";
}

// A target another program holds open (L7b0's review; the lead asked for
// the Windows behaviour by name). On Windows the reader here opens it
// without FILE_SHARE_DELETE, so the rename over it fails: the save is
// refused, the temporary removed, and the target keeps its bytes. Elsewhere
// rename(2) replaces the name, and the reader keeps what it opened.
TEST(MeshFile, ATargetHeldOpenIsReplacedOrRefusedCleanly) {
    const TempDir dir("mesh-file-held");
    const MeshPtr mesh = built(cube({0.0, 0.0, 0.0}, 0.2, MeshUnit::metre));
    ASSERT_NE(mesh, nullptr);
    const std::filesystem::path target = dir.path() / "held.spmesh";
    {
        std::ofstream out(target, std::ios::binary);
        out << "held open";
    }
    std::ifstream reader(target, std::ios::binary);
    ASSERT_TRUE(reader.is_open());

    const spade::Result<void> saved = spade::save_mesh_file(target, *mesh);
    std::filesystem::path tmp = target;
    tmp += ".tmp-spade";
    EXPECT_FALSE(std::filesystem::exists(tmp)) << "the temporary outlived the save";
    const std::string seen((std::istreambuf_iterator<char>(reader)), std::istreambuf_iterator<char>());
    EXPECT_EQ(seen, "held open") << "the reader keeps what it opened";
    reader.close();
#ifdef _WIN32
    ASSERT_FALSE(saved.has_value());
    EXPECT_EQ(saved.error().code, Code::io_error);
    EXPECT_TRUE(contains(saved.error().context, "cannot be replaced")) << saved.error().context;
    const std::vector<uint8_t> kept = read_bytes(target);
    EXPECT_EQ(std::string(kept.begin(), kept.end()), "held open");
#else
    ASSERT_TRUE(saved.has_value()) << saved.error().context;
    EXPECT_TRUE(spade::load_mesh_file(target, mesh->content_hash()).has_value());
#endif
}

}  // namespace
