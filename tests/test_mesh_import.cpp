#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_import.hpp"

// ---------------------------------------------------------------------------
// world/mesh_import.hpp: the glTF geometry reader (the parts and meshes plan,
// Task B6; CORE-6). Every fixture is built here, as JSON text with a base64
// buffer or as a .glb, from the unit cube [0, 1]^3 whose corners are
// numbered x + 2y + 4z (test_mesh.cpp's), so each expected position can be
// read off the fixture.
// ---------------------------------------------------------------------------

namespace {

using Json = nlohmann::json;
using spade::Code;
using spade::MeshObject;
using spade::TriangleMesh;

constexpr uint32_t kUByte = 5121;
constexpr uint32_t kUShort = 5123;
constexpr uint32_t kUInt = 5125;
constexpr uint32_t kFloat = 5126;

// The cube's twelve triangles, counter-clockwise seen from outside.
constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // z = 0, normal -z
    {4, 5, 6}, {5, 7, 6},  // z = 1, normal +z
    {0, 1, 4}, {1, 5, 4},  // y = 0, normal -y
    {2, 6, 3}, {3, 6, 7},  // y = 1, normal +y
    {0, 4, 2}, {2, 4, 6},  // x = 0, normal -x
    {1, 3, 5}, {3, 7, 5},  // x = 1, normal +x
};

glm::vec3 corner(uint32_t v) {
    return {static_cast<float>(v & 1u), static_cast<float>((v >> 1) & 1u), static_cast<float>((v >> 2) & 1u)};
}

std::vector<glm::vec3> cube_corners() {
    std::vector<glm::vec3> p;
    for (uint32_t v = 0; v < 8; ++v) p.push_back(corner(v));
    return p;
}

// Triangles [first, last) of the cube, as indices into its eight corners.
std::vector<uint32_t> cube_indices(std::size_t first = 0, std::size_t last = 12) {
    std::vector<uint32_t> idx;
    for (std::size_t t = first; t < last; ++t) {
        idx.insert(idx.end(), {kCubeTriangles[t][0], kCubeTriangles[t][1], kCubeTriangles[t][2]});
    }
    return idx;
}

// The corners in first-use order: the order the triangles first name them.
std::vector<uint32_t> first_use_corners() {
    std::vector<uint32_t> order;
    for (const auto& t : kCubeTriangles) {
        for (const uint32_t c : t) {
            if (std::find(order.begin(), order.end(), c) == order.end()) order.push_back(c);
        }
    }
    return order;
}

// --- little-endian bytes and base64 ----------------------------------------

void put_u8(std::vector<std::byte>& b, uint32_t v) {
    b.push_back(static_cast<std::byte>(v & 0xFFu));
}
void put_u16(std::vector<std::byte>& b, uint32_t v) {
    put_u8(b, v);
    put_u8(b, v >> 8);
}
void put_u32(std::vector<std::byte>& b, uint32_t v) {
    put_u16(b, v & 0xFFFFu);
    put_u16(b, v >> 16);
}
void put_f32(std::vector<std::byte>& b, float f) {
    put_u32(b, std::bit_cast<uint32_t>(f));
}
void pad4(std::vector<std::byte>& b, std::byte fill) {
    while (b.size() % 4 != 0) b.push_back(fill);
}

std::string base64(std::span<const std::byte> bytes) {
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); i += 3) {
        uint32_t n = std::to_integer<uint32_t>(bytes[i]) << 16;
        if (i + 1 < bytes.size()) n |= std::to_integer<uint32_t>(bytes[i + 1]) << 8;
        if (i + 2 < bytes.size()) n |= std::to_integer<uint32_t>(bytes[i + 2]);
        out += kAlphabet[(n >> 18) & 63u];
        out += kAlphabet[(n >> 12) & 63u];
        out += i + 1 < bytes.size() ? kAlphabet[(n >> 6) & 63u] : '=';
        out += i + 2 < bytes.size() ? kAlphabet[n & 63u] : '=';
    }
    return out;
}

std::vector<std::byte> bytes_of(std::string_view s) {
    std::vector<std::byte> b(s.size());
    if (!s.empty()) std::memcpy(b.data(), s.data(), s.size());
    return b;
}

// --- a glTF document under construction: one buffer, views 4-aligned -------

class Gltf {
public:
    Gltf() {
        doc["asset"] = Json::object();
        doc["asset"]["version"] = "2.0";
        doc["accessors"] = Json::array();
        doc["bufferViews"] = Json::array();
        doc["meshes"] = Json::array();
        doc["nodes"] = Json::array();
    }

    Json doc;
    std::vector<std::byte> bin;

    uint64_t view(const std::vector<std::byte>& bytes) {
        pad4(bin, std::byte{0});
        Json v = Json::object();
        v["buffer"] = 0;
        v["byteOffset"] = bin.size();
        v["byteLength"] = bytes.size();
        bin.insert(bin.end(), bytes.begin(), bytes.end());
        doc["bufferViews"].push_back(v);
        return doc["bufferViews"].size() - 1;
    }

    uint64_t positions(const std::vector<glm::vec3>& p) {
        std::vector<std::byte> b;
        glm::vec3 lo = p[0];
        glm::vec3 hi = p[0];
        for (const glm::vec3& q : p) {
            put_f32(b, q.x);
            put_f32(b, q.y);
            put_f32(b, q.z);
            lo = glm::min(lo, q);
            hi = glm::max(hi, q);
        }
        Json a = Json::object();
        a["bufferView"] = view(b);
        a["componentType"] = kFloat;
        a["count"] = p.size();
        a["type"] = "VEC3";
        a["min"] = {lo.x, lo.y, lo.z};
        a["max"] = {hi.x, hi.y, hi.z};
        doc["accessors"].push_back(a);
        return doc["accessors"].size() - 1;
    }

    uint64_t indices(const std::vector<uint32_t>& idx, uint32_t component_type) {
        std::vector<std::byte> b;
        for (const uint32_t i : idx) {
            if (component_type == kUByte) put_u8(b, i);
            if (component_type == kUShort) put_u16(b, i);
            if (component_type == kUInt) put_u32(b, i);
        }
        Json a = Json::object();
        a["bufferView"] = view(b);
        a["componentType"] = component_type;
        a["count"] = idx.size();
        a["type"] = "SCALAR";
        doc["accessors"].push_back(a);
        return doc["accessors"].size() - 1;
    }

    uint64_t mesh(const std::vector<Json>& primitives, const std::string& name) {
        Json m = Json::object();
        if (!name.empty()) m["name"] = name;
        m["primitives"] = primitives;
        doc["meshes"].push_back(m);
        return doc["meshes"].size() - 1;
    }

    uint64_t node(const Json& n) {
        doc["nodes"].push_back(n);
        return doc["nodes"].size() - 1;
    }

    void scene(const std::vector<uint64_t>& roots) {
        Json s = Json::object();
        s["nodes"] = roots;
        doc["scenes"] = Json::array();
        doc["scenes"].push_back(s);
        doc["scene"] = 0;
    }

    // The document, its buffer named by `uri`.
    [[nodiscard]] Json with_uri(const std::string& uri) const {
        Json d = doc;
        Json b = Json::object();
        b["byteLength"] = bin.size();
        b["uri"] = uri;
        d["buffers"] = Json::array();
        d["buffers"].push_back(b);
        return d;
    }

    // The document, its buffer a base64 data: URI.
    [[nodiscard]] Json embedded() const {
        return with_uri("data:application/octet-stream;base64," + base64(bin));
    }
    [[nodiscard]] std::string text() const { return embedded().dump(); }

    // The document in a .glb container: buffer 0 is the BIN chunk.
    [[nodiscard]] std::vector<std::byte> glb() const {
        Json d = doc;
        Json b = Json::object();
        b["byteLength"] = bin.size();
        d["buffers"] = Json::array();
        d["buffers"].push_back(b);
        std::string json = d.dump();
        while (json.size() % 4 != 0) json += ' ';
        std::vector<std::byte> body = bin;
        pad4(body, std::byte{0});
        std::vector<std::byte> out;
        put_u32(out, 0x46546C67u);  // "glTF"
        put_u32(out, 2);
        put_u32(out, static_cast<uint32_t>(12 + 8 + json.size() + 8 + body.size()));
        put_u32(out, static_cast<uint32_t>(json.size()));
        put_u32(out, 0x4E4F534Au);  // "JSON"
        const std::vector<std::byte> j = bytes_of(json);
        out.insert(out.end(), j.begin(), j.end());
        put_u32(out, static_cast<uint32_t>(body.size()));
        put_u32(out, 0x004E4942u);  // "BIN\0"
        out.insert(out.end(), body.begin(), body.end());
        return out;
    }
};

Json primitive(uint64_t position, std::optional<uint64_t> indices) {
    Json p = Json::object();
    p["attributes"] = Json::object();
    p["attributes"]["POSITION"] = position;
    if (indices) p["indices"] = *indices;
    return p;
}

// One cube mesh ("cube": eight corners, `component_type` indices) under one
// node holding `node_fields`, the scene's only root.
Gltf one_cube(Json node_fields, uint32_t component_type = kUShort) {
    Gltf g;
    const uint64_t p = g.positions(cube_corners());
    const uint64_t i = g.indices(cube_indices(), component_type);
    node_fields["mesh"] = g.mesh({primitive(p, i)}, "cube");
    g.scene({g.node(node_fields)});
    return g;
}

// The cube mesh under a node translated (1, 2, 3) and turned 90 degrees
// about +Y, whose parent is scaled 2.
Gltf cube_under_two_nodes() {
    Gltf g;
    const uint64_t p = g.positions(cube_corners());
    const uint64_t i = g.indices(cube_indices(), kUShort);
    const uint64_t m = g.mesh({primitive(p, i)}, "cube");
    const double h = 0.7071067811865476;  // sin 45 deg = cos 45 deg
    Json child = Json::object();
    child["name"] = "cube_node";
    child["translation"] = {1.0, 2.0, 3.0};
    child["rotation"] = {0.0, h, 0.0, h};
    child["mesh"] = m;
    const uint64_t c = g.node(child);
    Json parent = Json::object();
    parent["name"] = "parent";
    parent["scale"] = {2.0, 2.0, 2.0};
    parent["children"] = Json::array({c});
    g.scene({g.node(parent)});
    return g;
}

spade::Result<std::vector<MeshObject>> read_text(const std::string& text, const std::filesystem::path& base = {}) {
    const std::vector<std::byte> b = bytes_of(text);
    return spade::read_gltf_bytes(b, base);
}

bool has_vertex(const TriangleMesh& m, glm::vec3 q, float tol) {
    for (const glm::vec3& p : m.positions()) {
        if (std::abs(p.x - q.x) <= tol && std::abs(p.y - q.y) <= tol && std::abs(p.z - q.z) <= tol) return true;
    }
    return false;
}

// The signed volume, in double, by the divergence theorem: an independent
// check that the winding points outward.
double signed_volume(const TriangleMesh& m) {
    const auto p = m.positions();
    const auto idx = m.indices();
    double v = 0.0;
    for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
        const glm::dvec3 a(p[idx[t]]);
        const glm::dvec3 b(p[idx[t + 1]]);
        const glm::dvec3 c(p[idx[t + 2]]);
        v += glm::dot(a, glm::cross(b, c)) / 6.0;
    }
    return v;
}

void expect_same_mesh(const TriangleMesh& a, const TriangleMesh& b) {
    ASSERT_EQ(a.positions().size(), b.positions().size());
    EXPECT_EQ(std::memcmp(a.positions().data(), b.positions().data(), a.positions().size_bytes()), 0);
    EXPECT_TRUE(std::equal(a.indices().begin(), a.indices().end(), b.indices().begin(), b.indices().end()));
    EXPECT_EQ(a.content_hash(), b.content_hash());
}

void write_file(const std::filesystem::path& path, std::span<const std::byte> bytes) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

bool contains(const std::string& haystack, std::string_view needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

TEST(MeshImport, GltfNodeTransformsAreBakedInMetres) {
    // A unit cube mesh under a node translated (1, 2, 3) and rotated 90 degrees about +Y,
    // whose parent is scaled 2: vertex (1, 0, 0) lands at 2 * ((1, 2, 3) + (0, 0, -1)).
    const auto objects = read_text(cube_under_two_nodes().text());
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 1u);
    EXPECT_EQ((*objects)[0].name, "cube_node");
    const TriangleMesh& m = *(*objects)[0].mesh;
    EXPECT_EQ(m.positions().size(), 8u);
    EXPECT_EQ(m.triangle_count(), 12u);
    EXPECT_TRUE(has_vertex(m, glm::vec3(2.0f, 4.0f, 4.0f), 1e-6f));
    EXPECT_TRUE(has_vertex(m, glm::vec3(2.0f, 4.0f, 6.0f), 1e-6f));  // the origin: 2 * (1, 2, 3)
    EXPECT_TRUE(has_vertex(m, glm::vec3(4.0f, 6.0f, 4.0f), 1e-6f));  // (1, 1, 1): 2 * ((1, 2, 3) + (1, 1, -1))
}

TEST(MeshImport, GlbReadsAsTheSameObjects) {
    // The same document in a .glb container, in memory and from a file: the
    // same objects, byte for byte.
    const Gltf g = cube_under_two_nodes();
    const auto text = read_text(g.text());
    const std::vector<std::byte> glb = g.glb();
    const auto bytes = spade::read_gltf_bytes(glb, {});
    const spade::test::TempDir dir("mesh-import-glb");
    write_file(dir.path() / "cube.glb", glb);
    const auto file = spade::read_gltf(dir.path() / "cube.glb");
    ASSERT_TRUE(text.has_value()) << text.error().context;
    ASSERT_TRUE(bytes.has_value()) << bytes.error().context;
    ASSERT_TRUE(file.has_value()) << file.error().context;
    ASSERT_EQ(text->size(), 1u);
    ASSERT_EQ(bytes->size(), 1u);
    ASSERT_EQ(file->size(), 1u);
    EXPECT_EQ((*bytes)[0].name, (*text)[0].name);
    EXPECT_EQ((*file)[0].name, (*text)[0].name);
    expect_same_mesh(*(*bytes)[0].mesh, *(*text)[0].mesh);
    expect_same_mesh(*(*file)[0].mesh, *(*text)[0].mesh);
}

TEST(MeshImport, AMirroredNodeKeepsItsOrientationOutward) {
    // scale (-1, 1, 1) turns every triangle clockwise; the reader reverses
    // the winding, so make_mesh() accepts it and the volume is +1.
    Json n = Json::object();
    n["name"] = "mirrored";
    n["scale"] = {-1.0, 1.0, 1.0};
    const auto objects = read_text(one_cube(n).text());
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 1u);
    const TriangleMesh& m = *(*objects)[0].mesh;
    EXPECT_TRUE(has_vertex(m, glm::vec3(-1.0f, 0.0f, 0.0f), 0.0f));
    EXPECT_FALSE(has_vertex(m, glm::vec3(1.0f, 0.0f, 0.0f), 0.0f));
    EXPECT_NEAR(signed_volume(m), 1.0, 1e-12);
}

TEST(MeshImport, GltfSeamsAreWeldedSoAClosedMeshStaysClosed) {
    // A cube as exporters write one for split normals: each face's two
    // triangles have their own four vertices, 24 in all. Exact welding makes
    // it the closed 8-vertex cube again, numbered in first-use order.
    std::vector<glm::vec3> pos;
    std::vector<uint32_t> idx;
    for (std::size_t f = 0; f < 6; ++f) {
        const uint32_t base = static_cast<uint32_t>(pos.size());
        std::vector<uint32_t> local;
        for (std::size_t t = 2 * f; t < 2 * f + 2; ++t) {
            for (const uint32_t c : kCubeTriangles[t]) {
                const auto it = std::find(local.begin(), local.end(), c);
                const uint32_t k = static_cast<uint32_t>(it - local.begin());
                if (it == local.end()) {
                    local.push_back(c);
                    pos.push_back(corner(c));
                }
                idx.push_back(base + k);
            }
        }
    }
    ASSERT_EQ(pos.size(), 24u);
    Gltf g;
    const uint64_t p = g.positions(pos);
    const uint64_t i = g.indices(idx, kUShort);
    Json n = Json::object();
    n["name"] = "seamed";
    n["mesh"] = g.mesh({primitive(p, i)}, "");
    g.scene({g.node(n)});

    const auto objects = read_text(g.text());
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 1u);
    const TriangleMesh& m = *(*objects)[0].mesh;
    ASSERT_EQ(m.positions().size(), 8u);
    EXPECT_EQ(m.triangle_count(), 12u);
    EXPECT_EQ(m.shell_count(), 1u);
    const std::vector<uint32_t> order = first_use_corners();
    for (std::size_t k = 0; k < 8; ++k) {
        EXPECT_EQ(m.positions()[k], corner(order[k])) << "vertex " << k;
    }
}

TEST(MeshImport, UnsupportedGltfFeaturesAreUnavailableByName) {
    struct Case {
        const char* why;
        void (*edit)(Json&);
        const char* needle;
        const char* element;
    };
    const Case cases[] = {
        {"Draco, required",
         [](Json& d) {
             d["extensionsUsed"] = Json::array({"KHR_draco_mesh_compression"});
             d["extensionsRequired"] = Json::array({"KHR_draco_mesh_compression"});
         },
         "KHR_draco_mesh_compression", "extension"},
        {"meshopt, required",
         [](Json& d) {
             d["extensionsUsed"] = Json::array({"EXT_meshopt_compression"});
             d["extensionsRequired"] = Json::array({"EXT_meshopt_compression"});
         },
         "EXT_meshopt_compression", "extension"},
        {"quantized meshes, required",
         [](Json& d) {
             d["extensionsUsed"] = Json::array({"KHR_mesh_quantization"});
             d["extensionsRequired"] = Json::array({"KHR_mesh_quantization"});
         },
         "KHR_mesh_quantization", "extension"},
        {"mode 1, LINES", [](Json& d) { d["meshes"][0]["primitives"][0]["mode"] = 1; }, "LINES",
         "mesh 0 primitive 0"},
        {"mode 0, POINTS", [](Json& d) { d["meshes"][0]["primitives"][0]["mode"] = 0; }, "POINTS",
         "mesh 0 primitive 0"},
        {"mode 5, TRIANGLE_STRIP", [](Json& d) { d["meshes"][0]["primitives"][0]["mode"] = 5; },
         "TRIANGLE_STRIP", "mesh 0 primitive 0"},
        {"mode 6, TRIANGLE_FAN", [](Json& d) { d["meshes"][0]["primitives"][0]["mode"] = 6; }, "TRIANGLE_FAN",
         "mesh 0 primitive 0"},
        {"a sparse accessor",
         [](Json& d) {
             Json s = Json::object();
             s["count"] = 1;
             s["indices"] = Json::object();
             s["indices"]["bufferView"] = 1;
             s["indices"]["componentType"] = 5125;
             s["values"] = Json::object();
             s["values"]["bufferView"] = 0;
             d["accessors"][1]["sparse"] = s;
         },
         "sparse", "accessor 1"},
        {"a quantized POSITION", [](Json& d) { d["accessors"][0]["componentType"] = 5123; }, "VEC3 FLOAT",
         "accessor 0"},
        {"morph targets",
         [](Json& d) {
             Json t = Json::object();
             t["POSITION"] = 0;
             d["meshes"][0]["primitives"][0]["targets"] = Json::array({t});
         },
         "morph target", "mesh 0 primitive 0"},
        {"a skinned node", [](Json& d) { d["nodes"][0]["skin"] = 0; }, "skin", "node 0"},
    };
    for (const Case& c : cases) {
        Json d = one_cube(Json::object()).embedded();
        c.edit(d);
        const auto objects = read_text(d.dump());
        ASSERT_FALSE(objects.has_value()) << c.why;
        EXPECT_EQ(objects.error().code, Code::unavailable) << c.why << ": " << objects.error().context;
        EXPECT_TRUE(contains(objects.error().context, c.needle)) << c.why << ": " << objects.error().context;
        EXPECT_TRUE(contains(objects.error().context, c.element)) << c.why << ": " << objects.error().context;
        EXPECT_TRUE(contains(objects.error().context, "(glTF bytes): ")) << c.why << ": " << objects.error().context;
    }
}

TEST(MeshImport, AnOpenGltfObjectIsRefusedByItsName) {
    // A cube with its last triangle missing, in a .gltf with a UTF-8 name
    // and an external .bin whose URI is percent-encoded: refused with
    // make_mesh()'s own message, naming the file (as UTF-8) and the object.
    Gltf g;
    const uint64_t p = g.positions(cube_corners());
    const uint64_t i = g.indices(cube_indices(0, 11), kUInt);
    Json n = Json::object();
    n["name"] = "lid_off";
    n["mesh"] = g.mesh({primitive(p, i)}, "cube");
    g.scene({g.node(n)});

    const spade::test::TempDir dir("mesh-import-open");
    // "wuerfel.gltf" with a u-umlaut, as UTF-8.
    const std::filesystem::path file = dir.path() / std::filesystem::path(u8"w\xC3\xBC" u8"rfel.gltf");
    write_file(dir.path() / "lid off.bin", g.bin);
    write_file(file, bytes_of(g.with_uri("lid%20off.bin").dump()));

    const auto objects = spade::read_gltf(file);
    ASSERT_FALSE(objects.has_value());
    EXPECT_EQ(objects.error().code, Code::invalid_argument) << objects.error().context;
    EXPECT_TRUE(contains(objects.error().context, "w\xC3\xBC" "rfel.gltf: object 'lid_off': "))
        << objects.error().context;
    EXPECT_TRUE(contains(objects.error().context, "open edges")) << objects.error().context;
}

TEST(MeshImport, EachMeshNodeIsAnObjectInSceneOrder) {
    Gltf g;
    // Mesh 0, "cube_mesh": two primitives, each half the cube with its own
    // POSITION accessor; the second's has a ninth vertex no triangle names.
    std::vector<glm::vec3> nine = cube_corners();
    nine.push_back(glm::vec3(5.0f, 5.0f, 5.0f));
    const uint64_t p0 = g.positions(cube_corners());
    const uint64_t i0 = g.indices(cube_indices(0, 6), kUInt);
    const uint64_t p1 = g.positions(nine);
    const uint64_t i1 = g.indices(cube_indices(6, 12), kUInt);
    const uint64_t m0 = g.mesh({primitive(p0, i0), primitive(p1, i1)}, "cube_mesh");
    // Mesh 1, unnamed: one primitive, UNSIGNED_BYTE indices.
    const uint64_t m1 = g.mesh({primitive(p0, g.indices(cube_indices(), kUByte))}, "");

    Json a = Json::object();  // node 0: an empty name, so it takes its mesh's
    a["name"] = "";
    a["mesh"] = m0;
    a["translation"] = {10.0, 0.0, 0.0};
    Json b = Json::object();  // node 1: no names anywhere, so "node[1]"; a matrix moving +5 in y
    b["mesh"] = m1;
    b["matrix"] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5.0, 0.0, 1.0};
    Json c = Json::object();  // node 2: the assembly, no mesh, children in the order 1, 0
    c["name"] = "assembly";
    const uint64_t na = g.node(a);
    const uint64_t nb = g.node(b);
    c["children"] = Json::array({nb, na});
    const uint64_t nc = g.node(c);
    Json d = Json::object();  // node 3: only in scene 0, which is not the default
    d["name"] = "elsewhere";
    d["mesh"] = m1;
    const uint64_t nd = g.node(d);
    Json s0 = Json::object();
    s0["nodes"] = Json::array({nd});
    Json s1 = Json::object();
    s1["nodes"] = Json::array({nc});
    g.doc["scenes"] = Json::array({s0, s1});
    g.doc["scene"] = 1;

    const auto objects = read_text(g.text());
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 2u);
    EXPECT_EQ((*objects)[0].name, "node[1]");
    EXPECT_EQ((*objects)[1].name, "cube_mesh");

    const TriangleMesh& first = *(*objects)[0].mesh;
    EXPECT_EQ(first.positions().size(), 8u);
    EXPECT_TRUE(has_vertex(first, glm::vec3(0.0f, 5.0f, 0.0f), 0.0f));
    EXPECT_TRUE(has_vertex(first, glm::vec3(1.0f, 6.0f, 1.0f), 0.0f));

    const TriangleMesh& second = *(*objects)[1].mesh;
    EXPECT_EQ(second.positions().size(), 8u) << "joined across primitives, welded, the unnamed vertex dropped";
    EXPECT_EQ(second.triangle_count(), 12u);
    EXPECT_EQ(second.shell_count(), 1u);
    EXPECT_TRUE(has_vertex(second, glm::vec3(10.0f, 0.0f, 0.0f), 0.0f));
    EXPECT_TRUE(has_vertex(second, glm::vec3(11.0f, 1.0f, 1.0f), 0.0f));
    EXPECT_FALSE(has_vertex(second, glm::vec3(15.0f, 5.0f, 5.0f), 0.0f));
}

TEST(MeshImport, EveryIndexTypeReadsAlike) {
    // UNSIGNED_BYTE, UNSIGNED_SHORT and UNSIGNED_INT indices, and none (a
    // triangle soup of 36 vertices), give the same mesh, hash for hash.
    std::vector<uint64_t> hashes;
    for (const uint32_t type : {kUByte, kUShort, kUInt}) {
        Json n = Json::object();
        n["name"] = "cube";
        const auto objects = read_text(one_cube(n, type).text());
        ASSERT_TRUE(objects.has_value()) << type << ": " << objects.error().context;
        ASSERT_EQ(objects->size(), 1u);
        EXPECT_EQ((*objects)[0].mesh->positions().size(), 8u);
        hashes.push_back((*objects)[0].mesh->content_hash());
    }
    std::vector<glm::vec3> soup;
    for (const uint32_t v : cube_indices()) soup.push_back(corner(v));
    Gltf g;
    Json n = Json::object();
    n["name"] = "cube";
    n["mesh"] = g.mesh({primitive(g.positions(soup), std::nullopt)}, "");
    g.scene({g.node(n)});
    const auto objects = read_text(g.text());
    ASSERT_TRUE(objects.has_value()) << objects.error().context;
    ASSERT_EQ(objects->size(), 1u);
    EXPECT_EQ((*objects)[0].mesh->positions().size(), 8u);
    hashes.push_back((*objects)[0].mesh->content_hash());
    for (const uint64_t h : hashes) EXPECT_EQ(h, hashes[0]);
}

TEST(MeshImport, MalformedGltfIsRefusedByElement) {
    struct Case {
        const char* why;
        void (*edit)(Json&);
        Code code;
        const char* needle;
    };
    // Accessor 0 is POSITION (8 vertices, bufferView 0), accessor 1 the
    // indices (36, bufferView 1); one buffer.
    const Case cases[] = {
        {"an index out of range", [](Json& d) { d["accessors"][0]["count"] = 7; }, Code::invalid_argument,
         "mesh 0 primitive 0 indices accessor 1"},
        {"an index count not a multiple of 3", [](Json& d) { d["accessors"][1]["count"] = 35; },
         Code::invalid_argument, "35 indices"},
        {"a missing buffer", [](Json& d) { d["bufferViews"][0]["buffer"] = 3; }, Code::invalid_argument,
         "buffer 3"},
        {"a buffer with no uri", [](Json& d) { d["buffers"][0].erase("uri"); }, Code::invalid_argument,
         "buffer 0"},
        {"a byte-length mismatch",
         [](Json& d) { d["buffers"][0]["byteLength"] = d["buffers"][0]["byteLength"].get<uint64_t>() + 4; },
         Code::invalid_argument, "byteLength"},
        {"an accessor past its bufferView", [](Json& d) { d["accessors"][0]["count"] = 9; },
         Code::invalid_argument, "accessor 0"},
        {"a bufferView past its buffer", [](Json& d) { d["bufferViews"][1]["byteLength"] = 4096; },
         Code::invalid_argument, "bufferView 1"},
        {"a mesh that does not exist", [](Json& d) { d["nodes"][0]["mesh"] = 4; }, Code::invalid_argument,
         "mesh 4"},
        {"a node reached twice", [](Json& d) { d["nodes"][0]["children"] = Json::array({0}); }, Code::invalid_argument,
         "node 0"},
        {"a matrix beside TRS",
         [](Json& d) {
             d["nodes"][0]["matrix"] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
             d["nodes"][0]["translation"] = {0, 0, 0};
         },
         Code::invalid_argument, "matrix"},
        {"no mesh node", [](Json& d) { d["nodes"][0].erase("mesh"); }, Code::invalid_argument, "no node"},
        {"a data URI not in base64",
         [](Json& d) { d["buffers"][0]["uri"] = "data:application/octet-stream,abc"; }, Code::invalid_argument,
         "base64"},
        {"a URI with a scheme", [](Json& d) { d["buffers"][0]["uri"] = "https://example.com/cube.bin"; },
         Code::unavailable, "https"},
        {"an external buffer and no base directory", [](Json& d) { d["buffers"][0]["uri"] = "cube.bin"; },
         Code::invalid_argument, "base directory"},
    };
    for (const Case& c : cases) {
        Json n = Json::object();
        n["name"] = "cube";
        Json d = one_cube(n).embedded();
        c.edit(d);
        const auto objects = read_text(d.dump());
        ASSERT_FALSE(objects.has_value()) << c.why;
        EXPECT_EQ(objects.error().code, c.code) << c.why << ": " << objects.error().context;
        EXPECT_TRUE(contains(objects.error().context, c.needle)) << c.why << ": " << objects.error().context;
        EXPECT_TRUE(contains(objects.error().context, "(glTF bytes): ")) << c.why << ": " << objects.error().context;
    }

    const auto not_json = read_text("{ \"asset\": ");
    ASSERT_FALSE(not_json.has_value());
    EXPECT_EQ(not_json.error().code, Code::invalid_argument) << not_json.error().context;

    // A .glb one byte short of its declared length.
    std::vector<std::byte> glb = one_cube(Json::object()).glb();
    glb.pop_back();
    const auto short_glb = spade::read_gltf_bytes(glb, {});
    ASSERT_FALSE(short_glb.has_value());
    EXPECT_EQ(short_glb.error().code, Code::invalid_argument) << short_glb.error().context;
    EXPECT_TRUE(contains(short_glb.error().context, "GLB")) << short_glb.error().context;

    // An external buffer that is not there.
    const spade::test::TempDir dir("mesh-import-missing");
    const auto missing = read_text(one_cube(Json::object()).with_uri("missing.bin").dump(), dir.path());
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, Code::io_error) << missing.error().context;
    EXPECT_TRUE(contains(missing.error().context, "missing.bin")) << missing.error().context;
}

TEST(MeshImport, TooManyTrianglesAreRefusedBeforeReading) {
    // 2^31 + 1 triangles of indices, as an accessor with no bufferView (all
    // zeros, so nothing is stored): refused from the count alone.
    Json n = Json::object();
    n["name"] = "cube";
    Json d = one_cube(n).embedded();
    d["accessors"][1].erase("bufferView");
    d["accessors"][1]["count"] = uint64_t{3} * ((uint64_t{1} << 31) + 1);
    const auto objects = read_text(d.dump());
    ASSERT_FALSE(objects.has_value());
    EXPECT_EQ(objects.error().code, Code::capacity_exceeded) << objects.error().context;
    EXPECT_TRUE(contains(objects.error().context, "object 'cube': 2147483649 triangles"))
        << objects.error().context;
}

// The step never links the readers (L5; the plan's Layering): no engine
// source outside the readers includes them, no engine target but
// spade_mesh_import links spade_mesh_import, and spade_world names no JSON
// parser. Green at red: a pin.
TEST(MeshImport, TheStepNeverLinksTheReaders) {
    namespace fs = std::filesystem;
    const fs::path engine(SPADE_ENGINE_DIR);
    int scanned = 0;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(engine)) {
        const std::string ext = entry.path().extension().string();
        if (!entry.is_regular_file() || (ext != ".hpp" && ext != ".cpp")) continue;
        const fs::path rel = entry.path().lexically_relative(engine);
        const bool reader = rel.generic_string().starts_with("world/mesh_import");
        ++scanned;
        std::ifstream in(entry.path());
        std::string line;
        int number = 0;
        while (std::getline(in, line)) {
            ++number;
            const std::size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos || reader) continue;
            constexpr std::string_view kReaders = "#include \"world/mesh_import";
            constexpr std::string_view kJson = "#include <nlohmann/";
            EXPECT_NE(line.compare(start, kReaders.size(), kReaders), 0)
                << rel.generic_string() << ":" << number << ": " << line;
            if (rel.generic_string().starts_with("world/")) {
                EXPECT_NE(line.compare(start, kJson.size(), kJson), 0)
                    << rel.generic_string() << ":" << number << ": " << line;
            }
        }
    }
    EXPECT_GT(scanned, 0);

    // Every target_link_libraries() call in engine/CMakeLists.txt that names
    // mesh_import is spade_mesh_import's own. Comments are stripped first.
    std::ifstream cmake(engine / "CMakeLists.txt");
    std::string text;
    for (std::string line; std::getline(cmake, line);) {
        text += line.substr(0, line.find('#'));
        text += ' ';
    }
    ASSERT_FALSE(text.empty());
    int calls = 0;
    for (std::size_t at = text.find("target_link_libraries("); at != std::string::npos;
         at = text.find("target_link_libraries(", at + 1)) {
        const std::size_t open = at + std::string_view("target_link_libraries").size();
        std::size_t close = open;
        int depth = 0;
        for (; close < text.size(); ++close) {
            if (text[close] == '(') ++depth;
            if (text[close] == ')' && --depth == 0) break;
        }
        const std::string call = text.substr(open + 1, close - open - 1);
        const std::size_t first = call.find_first_not_of(" \t\r\n");
        const std::string target = call.substr(first, call.find_first_of(" \t\r\n", first) - first);
        ++calls;
        if (contains(call, "mesh_import")) {
            EXPECT_EQ(target, "spade_mesh_import") << call;
        }
    }
    EXPECT_GT(calls, 0);
}
