// ---------------------------------------------------------------------------
// glTF 2.0 mesh loading -- S7a Task R4 (render/gltf.hpp).
//
// WHAT THIS FILE HAS TO PROVE:
//
//   THE REAL FIXTURE LOADS. tests/fixtures/render/gate-ring.gltf is a vendored
//   copy of content/meshes/track/gate-ring.gltf (the engine tree must not read
//   from content/ at runtime) -- one primitive, POSITION accessor count 256,
//   indices accessor count 384, both confirmed below by parsing the fixture's
//   own JSON directly (not trusting the loader to tell us its own input was
//   what we think it was).
//
//   FLAT NORMALS DUPLICATE VERTICES (task-R4-brief.md's own words: "requires
//   duplicating vertices per face"), and MeshData::normals's own doc comment
//   ("flat meshes duplicate vertices") makes this a struct-level invariant,
//   not a choice this loader made privately: the OUTPUT positions/normals/
//   indices are all sized to the file's INDEX count (384 = 128 triangles x 3
//   unique corners each), not its vertex count (256, which only existed
//   because the source file shared vertices across triangles -- sharing this
//   loader deliberately discards when it fabricates a normal per face).
//
//   REJECTION. One test per malformed shape, each asserting the CODE and, for
//   the ones this loader raises itself, the diagnostic -- "it returned an
//   error" is not the contract on its own.
//
//   NEVER A PARTIAL MESH. Every rejection test also implicitly proves this:
//   Result<MeshData>'s failure branch carries no MeshData at all to inspect.
//
//   DETERMINISM. Loading the same file twice yields byte-identical buffers.
//
//   BOTH BUFFER SOURCES. The fixture covers embedded base64; a hand-built
//   external-.bin case below covers the other half of requirement 4.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "render/gltf.hpp"
#include "render/scene.hpp"

namespace {

using Json = nlohmann::json;

using spade::Code;
using spade::Error;
using spade::Result;
using spade::render::load_gltf;
using spade::render::MeshData;
using spade::render::parse_gltf;

// Same GoogleTest plumbing as tests/test_world_file.cpp: a failed Result
// prints its code and context, which is the difference between "it failed"
// and "primitives[1]'s 'indices' accessor 7 is out of range".
template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                        << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

template <class T>
[[nodiscard]] int code_of(const Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}
[[nodiscard]] constexpr int code(Code c) { return static_cast<int>(c); }

template <class T>
[[nodiscard]] std::string why(const Result<T>& r) {
    return r ? std::string{} : r.error().context;
}

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

// SPADE_TESTS_DIR is baked in at configure time (tests/CMakeLists.txt) as an
// absolute path -- the kat testing spec's no-CWD rule, same discipline every
// other fixture-reading test in this target already follows.
[[nodiscard]] std::filesystem::path fixture_path(const char* name) {
    return std::filesystem::path(SPADE_TESTS_DIR) / "fixtures" / "render" / name;
}

// ===========================================================================
// Minimal single-triangle glTF builder -- one primitive, one embedded (via an
// external .bin the caller writes) VEC3/FLOAT POSITION accessor and one
// SCALAR/UNSIGNED_SHORT indices accessor, tightly packed, no interleaving.
// Returns the JSON object so callers can clone-and-mutate it for each
// malformed variant rather than editing fragile raw string literals.
//
// Geometry: a right triangle (0,0,0)-(1,0,0)-(0,1,0) in the XY plane --
// cross((1,0,0), (0,1,0)) = (0,0,1), a clean, non-degenerate face normal.
// ===========================================================================

struct TriangleBuffers {
    std::vector<uint8_t> position_bytes;  // 3 * vec3 float32, tightly packed
    std::vector<uint8_t> index_bytes;     // 3 * uint16, tightly packed
};

[[nodiscard]] TriangleBuffers make_triangle_buffers() {
    const float positions[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    const uint16_t indices[3] = {0, 1, 2};
    TriangleBuffers buffers;
    buffers.position_bytes.resize(sizeof(positions));
    std::memcpy(buffers.position_bytes.data(), positions, sizeof(positions));
    buffers.index_bytes.resize(sizeof(indices));
    std::memcpy(buffers.index_bytes.data(), indices, sizeof(indices));
    return buffers;
}

// Writes `bytes` to `path`, creating parent directories as needed.
void write_file(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(static_cast<bool>(out)) << "failed opening " << path.string() << " for writing";
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

// A fresh scratch directory per test (never reused -- avoids cross-test
// interference the way tests/test_world_file.cpp's own temp-file helpers do).
[[nodiscard]] std::filesystem::path scratch_dir(const char* unique_name) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "spade_gltf_test" / unique_name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

// Writes the standard triangle's positions+indices to `dir / "buffer.bin"`
// and returns the combined byte count -- for tests whose malformed condition
// lives entirely in the JSON (not the buffer bytes) but which still need
// POSITION to resolve successfully first (this loader validates/reads
// POSITION before it ever looks at `indices`), so a real, readable buffer
// file must exist even though its CONTENTS are not what the test is about.
[[nodiscard]] std::size_t write_default_triangle_buffer(const std::filesystem::path& dir) {
    const TriangleBuffers tri = make_triangle_buffers();
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);
    return combined.size();
}

// Builds a minimal valid glTF document referencing `bin_uri` as its one
// buffer, with the triangle's two bufferViews/accessors and one primitive.
// Callers mutate the returned object (add/remove/replace keys) to build
// malformed variants without hand-editing JSON text.
[[nodiscard]] Json make_triangle_gltf(const std::string& bin_uri, std::size_t byte_length) {
    Json doc;
    doc["asset"] = {{"version", "2.0"}};
    doc["buffers"] = Json::array({{{"uri", bin_uri}, {"byteLength", byte_length}}});
    doc["bufferViews"] = Json::array({
        {{"buffer", 0}, {"byteOffset", 0}, {"byteLength", 36}},
        {{"buffer", 0}, {"byteOffset", 36}, {"byteLength", 6}},
    });
    doc["accessors"] = Json::array({
        {{"bufferView", 0}, {"componentType", 5126}, {"count", 3}, {"type", "VEC3"}},
        {{"bufferView", 1}, {"componentType", 5123}, {"count", 3}, {"type", "SCALAR"}},
    });
    doc["meshes"] = Json::array({
        {{"primitives",
          Json::array({{{"attributes", {{"POSITION", 0}}}, {"indices", 1}, {"material", 0}, {"mode", 4}}})}},
    });
    return doc;
}

}  // namespace

// ===========================================================================
// The real fixture (embedded base64) -- the happy path.
// ===========================================================================

TEST(RenderGltf, FixtureAccessorCountsMatchTheCommittedFile) {
    // Independent of the loader entirely: parses the vendored fixture's own
    // JSON to pin the numbers task-R4-brief.md's "assert against those real
    // numbers" instruction names, so a future edit to the fixture cannot
    // silently invalidate what the rest of this file assumes about it.
    std::ifstream in(fixture_path("gate-ring.gltf"), std::ios::binary);
    ASSERT_TRUE(static_cast<bool>(in));
    const Json doc = Json::parse(in);
    ASSERT_EQ(doc["accessors"][0]["count"].get<int>(), 256);   // POSITION
    ASSERT_EQ(doc["accessors"][1]["count"].get<int>(), 384);   // indices
    ASSERT_EQ(doc["meshes"][0]["primitives"].size(), 1u);
}

TEST(RenderGltf, LoadsGateRingFixture) {
    const Result<MeshData> result = load_gltf(fixture_path("gate-ring.gltf"));
    ASSERT_OK(result);
    const MeshData& mesh = *result;

    // Flat-normal generation duplicates vertices per face: the file's index
    // count (384 -- 128 triangles) becomes the OUTPUT vertex count too, not
    // the file's original (shared) 256-vertex POSITION accessor.
    EXPECT_EQ(mesh.indices.size(), 384u);
    EXPECT_EQ(mesh.positions.size(), 384u);
    EXPECT_EQ(mesh.normals.size(), 384u);

    for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
        const glm::vec3& p = mesh.positions[i];
        EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) << "vertex " << i;
    }
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, mesh.positions.size());
    }
    // Sequential, unshared indices are exactly what per-face duplication
    // produces -- 0,1,2,3,4,5,...
    for (std::size_t i = 0; i < mesh.indices.size(); ++i) {
        EXPECT_EQ(mesh.indices[i], static_cast<uint32_t>(i));
    }
    // Every normal is a real, normalized face normal (not just finite).
    for (const glm::vec3& n : mesh.normals) {
        EXPECT_NEAR(glm::length(n), 1.0f, 1e-4f);
    }

    // One submesh (SR-11's NON-empty case -- this loader always emits
    // explicit arrays, never the tessellate.cpp-style implicit shorthand,
    // even for a single primitive): task-R4-brief.md's own framing of glTF as
    // "the first producer" of the non-empty case.
    ASSERT_EQ(mesh.submesh_first_index.size(), 1u);
    ASSERT_EQ(mesh.submesh_index_count.size(), 1u);
    ASSERT_EQ(mesh.submesh_material.size(), 1u);
    EXPECT_EQ(mesh.submesh_first_index[0], 0u);
    EXPECT_EQ(mesh.submesh_index_count[0], 384u);
    EXPECT_EQ(mesh.submesh_material[0], 0u);  // the fixture's primitive names material 0
}

TEST(RenderGltf, LoadingTwiceIsByteIdentical) {
    const Result<MeshData> a = load_gltf(fixture_path("gate-ring.gltf"));
    const Result<MeshData> b = load_gltf(fixture_path("gate-ring.gltf"));
    ASSERT_OK(a);
    ASSERT_OK(b);
    EXPECT_EQ(a->positions, b->positions);
    EXPECT_EQ(a->normals, b->normals);
    EXPECT_EQ(a->indices, b->indices);
    EXPECT_EQ(a->submesh_first_index, b->submesh_first_index);
    EXPECT_EQ(a->submesh_index_count, b->submesh_index_count);
    EXPECT_EQ(a->submesh_material, b->submesh_material);
}

TEST(RenderGltf, LoadGltfMissingFileReturnsIoError) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_gltf_test_no_such_file.gltf";
    std::filesystem::remove(path);
    const Result<MeshData> result = load_gltf(path);
    EXPECT_EQ(code_of(result), code(Code::io_error));
}

// ===========================================================================
// External .bin buffer -- requirement 4's other half (the fixture only
// exercises embedded base64).
// ===========================================================================

TEST(RenderGltf, ExternalBinFileLoadsSuccessfully) {
    const std::filesystem::path dir = scratch_dir("external_bin_happy");
    const TriangleBuffers tri = make_triangle_buffers();
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);

    const Json doc = make_triangle_gltf("buffer.bin", combined.size());
    const std::filesystem::path gltf_path = dir / "triangle.gltf";
    {
        std::ofstream out(gltf_path, std::ios::binary | std::ios::trunc);
        out << doc.dump();
    }

    const Result<MeshData> result = load_gltf(gltf_path);
    ASSERT_OK(result);
    const MeshData& mesh = *result;
    ASSERT_EQ(mesh.positions.size(), 3u);
    ASSERT_EQ(mesh.normals.size(), 3u);
    ASSERT_EQ(mesh.indices.size(), 3u);
    EXPECT_EQ(mesh.indices[0], 0u);
    EXPECT_EQ(mesh.indices[1], 1u);
    EXPECT_EQ(mesh.indices[2], 2u);
    EXPECT_NEAR(mesh.normals[0].x, 0.0f, 1e-6f);
    EXPECT_NEAR(mesh.normals[0].y, 0.0f, 1e-6f);
    EXPECT_NEAR(mesh.normals[0].z, 1.0f, 1e-6f);
    ASSERT_EQ(mesh.submesh_material.size(), 1u);
    EXPECT_EQ(mesh.submesh_material[0], 0u);
}

TEST(RenderGltf, ExternalBinFileMissingReturnsIoError) {
    const std::filesystem::path dir = scratch_dir("external_bin_missing");
    const Json doc = make_triangle_gltf("does_not_exist.bin", 42);
    const std::filesystem::path gltf_path = dir / "triangle.gltf";
    {
        std::ofstream out(gltf_path, std::ios::binary | std::ios::trunc);
        out << doc.dump();
    }
    const Result<MeshData> result = load_gltf(gltf_path);
    EXPECT_EQ(code_of(result), code(Code::io_error));
}

// ===========================================================================
// Rejection -- the brief's four named malformed cases, plus every other one
// this implementation found worth guarding while writing it.
// ===========================================================================

[[nodiscard]] std::filesystem::path base_dir_for_parse_only_tests() {
    // parse_gltf's base_dir is unused by every test below that never
    // references an external buffer -- any existing directory will do.
    return std::filesystem::temp_directory_path();
}

TEST(RenderGltf, NotJsonReturnsError) {
    const Result<MeshData> result = parse_gltf("this is not json at all {{{", base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, MissingMeshesReturnsError) {
    const Result<MeshData> result =
        parse_gltf(R"({"asset": {"version": "2.0"}})", base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
    EXPECT_TRUE(contains(why(result), "meshes")) << why(result);
}

TEST(RenderGltf, EmptyMeshesArrayReturnsError) {
    const Result<MeshData> result =
        parse_gltf(R"({"asset": {"version": "2.0"}, "meshes": []})", base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, EmptyPrimitivesArrayReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["meshes"][0]["primitives"] = Json::array();
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, OutOfRangeAccessorIndexReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["meshes"][0]["primitives"][0]["attributes"]["POSITION"] = 99;  // only 2 accessors exist
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
    EXPECT_TRUE(contains(why(result), "out of range")) << why(result);
}

TEST(RenderGltf, UnsupportedComponentTypeOnPositionReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["accessors"][0]["componentType"] = 5121;  // UNSIGNED_BYTE -- POSITION must be FLOAT
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
    EXPECT_TRUE(contains(why(result), "unsupported component type")) << why(result);
}

TEST(RenderGltf, UnsupportedComponentTypeOnIndicesReturnsError) {
    // POSITION is well-formed and must resolve successfully before this
    // primitive's malformed `indices` accessor is ever inspected, so a real
    // backing buffer is required even though its bytes are otherwise unused.
    const std::filesystem::path dir = scratch_dir("bad_index_component_type");
    const std::size_t size = write_default_triangle_buffer(dir);
    Json doc = make_triangle_gltf("buffer.bin", size);
    doc["accessors"][1]["componentType"] = 5126;  // FLOAT -- indices must be an unsigned int type
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
    EXPECT_TRUE(contains(why(result), "unsupported component type")) << why(result);
}

TEST(RenderGltf, MissingPositionAttributeReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["meshes"][0]["primitives"][0]["attributes"].erase("POSITION");
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, MissingIndicesReturnsError) {
    // Same reasoning as UnsupportedComponentTypeOnIndicesReturnsError above:
    // POSITION resolves first and needs a real buffer behind it.
    const std::filesystem::path dir = scratch_dir("missing_indices");
    const std::size_t size = write_default_triangle_buffer(dir);
    Json doc = make_triangle_gltf("buffer.bin", size);
    doc["meshes"][0]["primitives"][0].erase("indices");
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, IndexOutOfRangeReturnsError) {
    const std::filesystem::path dir = scratch_dir("index_out_of_range");
    const uint16_t bad_indices[3] = {0, 1, 5};  // vertex 5 does not exist (only 3 positions)
    TriangleBuffers tri = make_triangle_buffers();
    std::memcpy(tri.index_bytes.data(), bad_indices, sizeof(bad_indices));
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);

    const Json doc = make_triangle_gltf("buffer.bin", combined.size());
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, NonFinitePositionReturnsError) {
    const std::filesystem::path dir = scratch_dir("non_finite_position");
    TriangleBuffers tri = make_triangle_buffers();
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(tri.position_bytes.data(), &nan_value, sizeof(float));  // corrupt vertex 0's x
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);

    const Json doc = make_triangle_gltf("buffer.bin", combined.size());
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, DegenerateTriangleReturnsError) {
    const std::filesystem::path dir = scratch_dir("degenerate_triangle");
    const float collinear[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f};  // zero-area
    TriangleBuffers tri = make_triangle_buffers();
    std::memcpy(tri.position_bytes.data(), collinear, sizeof(collinear));
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);

    const Json doc = make_triangle_gltf("buffer.bin", combined.size());
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, IndexCountNotMultipleOfThreeReturnsError) {
    // Same reasoning again: POSITION resolves first and needs a real buffer.
    const std::filesystem::path dir = scratch_dir("index_count_not_multiple_of_three");
    const std::size_t size = write_default_triangle_buffer(dir);
    Json doc = make_triangle_gltf("buffer.bin", size);
    doc["accessors"][1]["count"] = 2;
    doc["bufferViews"][1]["byteLength"] = 4;
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, NonTriangleModeReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["meshes"][0]["primitives"][0]["mode"] = 1;  // LINES
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, SparseAccessorReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["accessors"][0]["sparse"] = {{"count", 1}};
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, InterleavedBufferViewReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["bufferViews"][0]["byteStride"] = 16;  // not the natural 12-byte VEC3/FLOAT stride
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, BufferWithNoUriReturnsError) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["buffers"][0].erase("uri");
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, UnsupportedDataUriReturnsError) {
    Json doc = make_triangle_gltf("data:image/png;base64,AAAA", 42);
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, InvalidBase64ReturnsError) {
    Json doc = make_triangle_gltf("data:application/octet-stream;base64,not-valid-base64!!!", 42);
    const Result<MeshData> result = parse_gltf(doc.dump(), base_dir_for_parse_only_tests());
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

TEST(RenderGltf, BufferByteLengthMismatchReturnsError) {
    const std::filesystem::path dir = scratch_dir("byte_length_mismatch");
    const TriangleBuffers tri = make_triangle_buffers();
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);

    // Declares one byte more than the file actually holds.
    const Json doc = make_triangle_gltf("buffer.bin", combined.size() + 1);
    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    EXPECT_EQ(code_of(result), code(Code::invalid_argument));
}

// ===========================================================================
// Multi-primitive: submesh order and per-primitive material grouping.
// ===========================================================================

TEST(RenderGltf, MultiplePrimitivesPreserveFileOrderAndOwnMaterial) {
    const std::filesystem::path dir = scratch_dir("multi_primitive");

    // Two disjoint triangles, each its own primitive with its own accessors,
    // sharing one buffer -- primitive[0] first, primitive[1] second, in that
    // exact byte order, so a wrongly-reordering implementation (e.g. keyed by
    // material rather than by array position) would be caught by the
    // first-primitive/second-primitive material assertions below.
    const float positions_a[9] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    const float positions_b[9] = {10.0f, 0.0f, 0.0f, 11.0f, 0.0f, 0.0f, 10.0f, 1.0f, 0.0f};
    const uint16_t indices_ab[6] = {0, 1, 2, 0, 1, 2};  // each primitive re-uses local indices 0..2

    std::vector<uint8_t> combined;
    const auto append = [&combined](const void* data, std::size_t size) {
        const auto* bytes = static_cast<const uint8_t*>(data);
        combined.insert(combined.end(), bytes, bytes + size);
    };
    const std::size_t pos_a_offset = combined.size();
    append(positions_a, sizeof(positions_a));
    const std::size_t pos_b_offset = combined.size();
    append(positions_b, sizeof(positions_b));
    const std::size_t idx_offset = combined.size();
    append(indices_ab, sizeof(indices_ab));
    write_file(dir / "buffer.bin", combined);

    Json doc;
    doc["asset"] = {{"version", "2.0"}};
    doc["buffers"] = Json::array({{{"uri", "buffer.bin"}, {"byteLength", combined.size()}}});
    doc["bufferViews"] = Json::array({
        {{"buffer", 0}, {"byteOffset", pos_a_offset}, {"byteLength", 36}},
        {{"buffer", 0}, {"byteOffset", pos_b_offset}, {"byteLength", 36}},
        {{"buffer", 0}, {"byteOffset", idx_offset}, {"byteLength", 6}},
        {{"buffer", 0}, {"byteOffset", idx_offset + 6}, {"byteLength", 6}},
    });
    doc["accessors"] = Json::array({
        {{"bufferView", 0}, {"componentType", 5126}, {"count", 3}, {"type", "VEC3"}},
        {{"bufferView", 1}, {"componentType", 5126}, {"count", 3}, {"type", "VEC3"}},
        {{"bufferView", 2}, {"componentType", 5123}, {"count", 3}, {"type", "SCALAR"}},
        {{"bufferView", 3}, {"componentType", 5123}, {"count", 3}, {"type", "SCALAR"}},
    });
    doc["meshes"] = Json::array({
        {{"primitives",
          Json::array({
              {{"attributes", {{"POSITION", 0}}}, {"indices", 2}, {"material", 5}, {"mode", 4}},
              {{"attributes", {{"POSITION", 1}}}, {"indices", 3}, {"material", 1}, {"mode", 4}},
          })}},
    });

    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    ASSERT_OK(result);
    const MeshData& mesh = *result;

    ASSERT_EQ(mesh.submesh_first_index.size(), 2u);
    ASSERT_EQ(mesh.submesh_index_count.size(), 2u);
    ASSERT_EQ(mesh.submesh_material.size(), 2u);

    // File order, not material order (5 then 1 -- ascending-by-material would
    // reverse this).
    EXPECT_EQ(mesh.submesh_material[0], 5u);
    EXPECT_EQ(mesh.submesh_material[1], 1u);

    EXPECT_EQ(mesh.submesh_first_index[0], 0u);
    EXPECT_EQ(mesh.submesh_index_count[0], 3u);
    EXPECT_EQ(mesh.submesh_first_index[1], 3u);
    EXPECT_EQ(mesh.submesh_index_count[1], 3u);

    // Ranges partition the index buffer (SR-11): together they cover exactly
    // [0, indices.size()) with no gap and no overlap.
    EXPECT_EQ(mesh.indices.size(), 6u);
    EXPECT_EQ(mesh.submesh_first_index[0] + mesh.submesh_index_count[0], mesh.submesh_first_index[1]);
    EXPECT_EQ(mesh.submesh_first_index[1] + mesh.submesh_index_count[1], mesh.indices.size());

    // Second primitive's vertices are its own (offset into a shared,
    // concatenated positions array), not primitive 0's.
    EXPECT_NEAR(mesh.positions[3].x, 10.0f, 1e-6f);
}

TEST(RenderGltf, PrimitiveWithoutMaterialDefaultsToZero) {
    Json doc = make_triangle_gltf("buffer.bin", 42);
    doc["meshes"][0]["primitives"][0].erase("material");
    const std::filesystem::path dir = scratch_dir("default_material");
    const TriangleBuffers tri = make_triangle_buffers();
    std::vector<uint8_t> combined = tri.position_bytes;
    combined.insert(combined.end(), tri.index_bytes.begin(), tri.index_bytes.end());
    write_file(dir / "buffer.bin", combined);
    doc["buffers"][0]["byteLength"] = combined.size();

    const Result<MeshData> result = parse_gltf(doc.dump(), dir);
    ASSERT_OK(result);
    ASSERT_EQ(result->submesh_material.size(), 1u);
    EXPECT_EQ(result->submesh_material[0], 0u);
}
