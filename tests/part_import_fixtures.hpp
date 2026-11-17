#pragma once

// Fixtures for spade_part_import's tests (L88): a closed box and the cube written as the formats the tool reads (binary STL, OBJ text,
// GLB, 3MF), byte by byte, with no network and no CAD kernel. The cube is test_mesh.cpp's: corners numbered x + 2y + 4z, twelve
// triangles counter-clockwise seen from outside.

#include <miniz.h>

#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <gtest/gtest.h>

#include "world/mesh.hpp"

namespace spade::partimporttest {

inline constexpr uint32_t kCube[12][3] = {
    {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
    {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
};

[[nodiscard]] inline glm::vec3 corner(uint32_t v, float s = 1.0f) {
    return {static_cast<float>(v & 1u) * s, static_cast<float>((v >> 1) & 1u) * s, static_cast<float>((v >> 2) & 1u) * s};
}

inline void write_file(const std::filesystem::path& path, std::string_view bytes) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

[[nodiscard]] inline std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// A closed box of sides sx, sy, sz with a corner at the origin, in metres.
[[nodiscard]] inline std::shared_ptr<const TriangleMesh> box_mesh(double sx, double sy, double sz) {
    MeshInput in;
    in.unit = MeshUnit::metre;
    for (uint32_t v = 0; v < 8; ++v) in.positions.push_back({(v & 1u) ? sx : 0.0, (v & 2u) ? sy : 0.0, (v & 4u) ? sz : 0.0});
    for (const auto& t : kCube) in.indices.insert(in.indices.end(), {t[0], t[1], t[2]});
    auto m = make_mesh(in, "box");
    EXPECT_TRUE(m.has_value()) << (m.has_value() ? "" : m.error().context);
    return m.has_value() ? *m : nullptr;
}

// The cube of side `s` as a binary STL soup, its first `faces` triangles.
[[nodiscard]] inline std::string cube_stl(float s = 1.0f, std::size_t faces = 12) {
    std::string b(80, '\0');
    const auto u32 = [&b](uint32_t v) {
        for (int k = 0; k < 4; ++k) b += static_cast<char>((v >> (8 * k)) & 0xFFu);
    };
    u32(static_cast<uint32_t>(faces));
    for (std::size_t i = 0; i < faces; ++i) {
        for (int k = 0; k < 3; ++k) u32(0);
        for (const uint32_t c : kCube[i]) {
            const glm::vec3 p = corner(c, s);
            u32(std::bit_cast<uint32_t>(p.x));
            u32(std::bit_cast<uint32_t>(p.y));
            u32(std::bit_cast<uint32_t>(p.z));
        }
        b += std::string(2, '\0');
    }
    return b;
}

// The cube of side `s` as OBJ text, its first `faces` triangles (fewer than twelve leaves it open).
[[nodiscard]] inline std::string cube_obj(float s = 1.0f, std::size_t faces = 12) {
    std::string t = "o cube\n";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v, s);
        t += "v " + std::to_string(c.x) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
    }
    for (std::size_t i = 0; i < faces; ++i) {
        t += "f " + std::to_string(kCube[i][0] + 1) + " " + std::to_string(kCube[i][1] + 1) + " " + std::to_string(kCube[i][2] + 1) + "\n";
    }
    return t;
}

// The cube of side `s` as a GLB with one node named `node_name`: positions and indices in the BIN chunk, metres by the specification.
[[nodiscard]] inline std::string cube_glb(float s = 1.0f, std::string_view node_name = "cube") {
    std::string bin;
    const auto f32 = [&bin](float v) {
        const auto bits = std::bit_cast<uint32_t>(v);
        for (int k = 0; k < 4; ++k) bin += static_cast<char>((bits >> (8 * k)) & 0xFFu);
    };
    const auto u32 = [&bin](uint32_t v) {
        for (int k = 0; k < 4; ++k) bin += static_cast<char>((v >> (8 * k)) & 0xFFu);
    };
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v, s);
        f32(c.x);
        f32(c.y);
        f32(c.z);
    }
    for (const auto& t : kCube) {
        for (const uint32_t i : t) u32(i);
    }
    std::string json = R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"name":")" + std::string(node_name) +
                       R"(","mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}],)"
                       R"("accessors":[{"bufferView":0,"componentType":5126,"count":8,"type":"VEC3"},)"
                       R"({"bufferView":1,"componentType":5125,"count":36,"type":"SCALAR"}],)"
                       R"("bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":96},{"buffer":0,"byteOffset":96,"byteLength":144}],)"
                       R"("buffers":[{"byteLength":240}]})";
    while (json.size() % 4 != 0) json += ' ';
    std::string out = "glTF";
    const auto le32 = [&out](uint32_t v) {
        for (int k = 0; k < 4; ++k) out += static_cast<char>((v >> (8 * k)) & 0xFFu);
    };
    le32(2);
    le32(static_cast<uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
    le32(static_cast<uint32_t>(json.size()));
    out += "JSON";
    out += json;
    le32(static_cast<uint32_t>(bin.size()));
    out += std::string("BIN\0", 4);
    out += bin;
    return out;
}

// A .gltf (JSON) that names a buffer file beside it: it needs a base directory, so the tool refuses it as not self-contained.
[[nodiscard]] inline std::string gltf_with_external_buffer() {
    return R"({"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"name":"cube","mesh":0}],)"
           R"("meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],)"
           R"("accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"}],)"
           R"("bufferViews":[{"buffer":0,"byteLength":36}],"buffers":[{"uri":"cube.bin","byteLength":36}]})";
}

// A <object> holding the cube scaled by `scale` and moved `dx` along x (a 3MF model's text).
[[nodiscard]] inline std::string cube_object(uint32_t id, std::string_view name, double scale = 1.0, double dx = 0.0, std::size_t faces = 12) {
    std::string s = "<object id=\"" + std::to_string(id) + "\" name=\"" + std::string(name) + "\"><mesh><vertices>";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v);
        s += "<vertex x=\"" + std::to_string(c.x * scale + dx) + "\" y=\"" + std::to_string(c.y * scale) + "\" z=\"" +
             std::to_string(c.z * scale) + "\"/>";
    }
    s += "</vertices><triangles>";
    for (std::size_t i = 0; i < faces; ++i) {
        s += "<triangle v1=\"" + std::to_string(kCube[i][0]) + "\" v2=\"" + std::to_string(kCube[i][1]) + "\" v3=\"" + std::to_string(kCube[i][2]) + "\"/>";
    }
    return s + "</triangles></mesh></object>";
}

// A 3MF package whose model holds `objects` (each built as an item, in order) under the root attributes `root`.
[[nodiscard]] inline std::string three_mf(const std::vector<std::string>& objects, std::string_view root = R"(unit="millimeter")") {
    std::string resources;
    std::string build;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        resources += objects[i];
        build += "<item objectid=\"" + std::to_string(i + 1) + "\"/>";
    }
    const std::string model = std::string(R"(<?xml version="1.0" encoding="UTF-8"?>)") +
                              R"(<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" )" + std::string(root) +
                              "><resources>" + resources + "</resources><build>" + build + "</build></model>";
    const std::string rels =
        R"(<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">)"
        R"(<Relationship Id="r0" Target="/3D/3dmodel.model" )"
        R"(Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>)";
    mz_zip_archive zip{};
    std::string out;
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) {
        ADD_FAILURE() << "mz_zip_writer_init_heap";
        return out;
    }
    for (const auto& [name, data] : {std::pair<const char*, const std::string*>{"_rels/.rels", &rels},
                                     std::pair<const char*, const std::string*>{"3D/3dmodel.model", &model}}) {
        if (!mz_zip_writer_add_mem(&zip, name, data->data(), data->size(), MZ_DEFAULT_LEVEL)) ADD_FAILURE() << "mz_zip_writer_add_mem " << name;
    }
    void* buf = nullptr;
    std::size_t size = 0;
    if (mz_zip_writer_finalize_heap_archive(&zip, &buf, &size)) {
        out.assign(static_cast<const char*>(buf), size);
    } else {
        ADD_FAILURE() << "mz_zip_writer_finalize_heap_archive";
    }
    mz_free(buf);
    mz_zip_writer_end(&zip);
    return out;
}

}  // namespace spade::partimporttest
