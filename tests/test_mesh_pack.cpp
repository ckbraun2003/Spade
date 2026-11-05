// test_mesh_pack.cpp -- tools/mesh_pack/mesh_pack.hpp, spade_mesh_pack's run()
// (CORE-6 D; the STEP converter plan, Task S1), compiled in from the tool.
// The fixtures are the unit cube, as a 3MF zipped with miniz, a binary STL
// or OBJ text, written here.
#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <miniz.h>

#include "../tools/mesh_pack/mesh_pack.hpp"
#include "unique_temp.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"

namespace {

namespace fs = std::filesystem;
using spade::MeshUnit;

// The cube's twelve triangles, counter-clockwise seen from outside, its
// corners numbered x + 2y + 4z (test_mesh.cpp's).
constexpr uint32_t kCube[12][3] = {
    {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
    {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
};

glm::vec3 corner(uint32_t v) {
    return {static_cast<float>(v & 1u), static_cast<float>((v >> 1) & 1u), static_cast<float>((v >> 2) & 1u)};
}

bool contains(const std::string& haystack, std::string_view needle) {
    return haystack.find(needle) != std::string::npos;
}

void write_file(const fs::path& path, std::string_view bytes) {
    std::ofstream out(path, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

// A <object> holding the cube scaled by `scale` and moved `dx` along x.
std::string cube_object(uint32_t id, std::string_view name, double scale = 1.0, double dx = 0.0) {
    std::string s = "<object id=\"" + std::to_string(id) + "\" name=\"" + std::string(name) + "\"><mesh><vertices>";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v);
        s += "<vertex x=\"" + std::to_string(c.x * scale + dx) + "\" y=\"" + std::to_string(c.y * scale) +
             "\" z=\"" + std::to_string(c.z * scale) + "\"/>";
    }
    s += "</vertices><triangles>";
    for (const auto& t : kCube) {
        s += "<triangle v1=\"" + std::to_string(t[0]) + "\" v2=\"" + std::to_string(t[1]) + "\" v3=\"" +
             std::to_string(t[2]) + "\"/>";
    }
    return s + "</triangles></mesh></object>";
}

// A 3MF package whose model holds `objects` (each built as an item, in
// order) under the root attributes `root`.
std::string three_mf(const std::vector<std::string>& objects, std::string_view root = R"(unit="millimeter")") {
    std::string resources;
    std::string build;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        resources += objects[i];
        build += "<item objectid=\"" + std::to_string(i + 1) + "\"/>";
    }
    const std::string model = std::string(R"(<?xml version="1.0" encoding="UTF-8"?>)") +
                              R"(<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" )" +
                              std::string(root) + "><resources>" + resources + "</resources><build>" + build +
                              "</build></model>";
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
        if (!mz_zip_writer_add_mem(&zip, name, data->data(), data->size(), MZ_DEFAULT_LEVEL)) {
            ADD_FAILURE() << "mz_zip_writer_add_mem " << name;
        }
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

// The cube as a binary STL soup.
std::string stl_cube() {
    std::string b(80, '\0');
    const auto u32 = [&b](uint32_t v) {
        for (int k = 0; k < 4; ++k) b += static_cast<char>((v >> (8 * k)) & 0xFFu);
    };
    u32(12);
    for (const auto& t : kCube) {
        for (int k = 0; k < 3; ++k) u32(0);
        for (const uint32_t c : t) {
            const glm::vec3 p = corner(c);
            u32(std::bit_cast<uint32_t>(p.x));
            u32(std::bit_cast<uint32_t>(p.y));
            u32(std::bit_cast<uint32_t>(p.z));
        }
        b += std::string(2, '\0');
    }
    return b;
}

// The cube as OBJ text, its first `faces` triangles.
std::string obj_cube(std::size_t faces = 12) {
    std::string s = "o cube\n";
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::vec3 c = corner(v);
        s += "v " + std::to_string(c.x) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
    }
    for (std::size_t t = 0; t < faces; ++t) {
        s += "f " + std::to_string(kCube[t][0] + 1) + " " + std::to_string(kCube[t][1] + 1) + " " +
             std::to_string(kCube[t][2] + 1) + "\n";
    }
    return s;
}

spade::mesh_pack::Outcome pack(std::initializer_list<fs::path> args) {
    const std::vector<fs::path> v(args);
    return spade::mesh_pack::run(v);
}

}  // namespace

TEST(MeshPack, PacksEachObjectOfA3mfWithItsProvenance) {
    // Two millimetre cubes of side 10, packed with a tolerance and a source:
    // each sidecar holds read_3mf()'s mesh, the model's unit, the tolerance
    // in metres and "<source>: <object>".
    const spade::test::TempDir dir("mesh-pack-3mf");
    const fs::path in = dir.path() / "bracket.3mf";
    write_file(in, three_mf({cube_object(1, "arm left", 10.0), cube_object(2, "hub", 10.0, 20.0)}));
    const fs::path out = dir.path() / "meshes";
    const auto run = pack({in, "--out", out, "--tolerance", "0.1mm", "--source",
                           "bracket.step (OCCT 7.9.3; deflection 1e-4 m)"});
    ASSERT_EQ(run.exit_code, 0) << run.err;
    EXPECT_TRUE(run.err.empty()) << run.err;

    const auto read = spade::read_3mf(in);
    ASSERT_TRUE(read.has_value()) << read.error().context;
    ASSERT_EQ(read->size(), 2u);
    for (const auto& [object, file] : {std::pair{0, "arm left.spmesh"}, std::pair{1, "hub.spmesh"}}) {
        const auto loaded = spade::load_mesh_file(out / file);
        ASSERT_TRUE(loaded.has_value()) << file << ": " << loaded.error().context;
        EXPECT_EQ(loaded->mesh->content_hash(), (*read)[object].mesh->content_hash()) << file;
        EXPECT_EQ(loaded->provenance.unit, MeshUnit::millimetre) << file;
        EXPECT_EQ(loaded->provenance.tolerance, 0.1 * 0.001) << file;
        EXPECT_EQ(loaded->provenance.source,
                  "bracket.step (OCCT 7.9.3; deflection 1e-4 m): " + (*read)[object].name)
            << file;
        EXPECT_TRUE(contains(run.out, (*read)[object].name + " -> ")) << run.out;
        EXPECT_TRUE(contains(run.out, file)) << run.out;
    }
}

TEST(MeshPack, TheSourceDefaultsToTheFileNameAndTheToleranceToNone) {
    const spade::test::TempDir dir("mesh-pack-default");
    const fs::path in = dir.path() / "cubes.3mf";
    write_file(in, three_mf({cube_object(1, "hub")}, R"(unit="inch")"));
    const auto run = pack({in, "--out", dir.path()});
    ASSERT_EQ(run.exit_code, 0) << run.err;
    const auto loaded = spade::load_mesh_file(dir.path() / "hub.spmesh");
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->provenance.source, "cubes.3mf: hub");
    EXPECT_EQ(loaded->provenance.unit, MeshUnit::inch);
    EXPECT_EQ(loaded->provenance.tolerance, 0.0);
}

TEST(MeshPack, StlAndObjTakeTheUnitTheCallerDeclares) {
    // STL and OBJ carry no unit, so --unit both reads them and is recorded;
    // without it the reader's own refusal comes back, exit 4.
    const spade::test::TempDir dir("mesh-pack-unit");
    write_file(dir.path() / "cube.stl", stl_cube());
    write_file(dir.path() / "cube.obj", obj_cube());
    const auto stl = pack({dir.path() / "cube.stl", "--out", dir.path() / "stl", "--unit", "mm"});
    ASSERT_EQ(stl.exit_code, 0) << stl.err;
    const auto stl_mesh = spade::load_mesh_file(dir.path() / "stl" / "cube.spmesh");
    ASSERT_TRUE(stl_mesh.has_value()) << stl_mesh.error().context;
    EXPECT_EQ(stl_mesh->provenance.unit, MeshUnit::millimetre);
    EXPECT_NEAR(stl_mesh->mesh->mass_properties().volume, 1e-9, 1e-15);

    const auto obj = pack({dir.path() / "cube.obj", "--out", dir.path() / "obj", "--unit", "in"});
    ASSERT_EQ(obj.exit_code, 0) << obj.err;
    const auto obj_mesh = spade::load_mesh_file(dir.path() / "obj" / "cube.spmesh");
    ASSERT_TRUE(obj_mesh.has_value()) << obj_mesh.error().context;
    EXPECT_EQ(obj_mesh->provenance.unit, MeshUnit::inch);
    EXPECT_EQ(obj_mesh->provenance.source, "cube.obj: cube");

    const auto bare = pack({dir.path() / "cube.stl", "--out", dir.path() / "bare"});
    EXPECT_EQ(bare.exit_code, 4) << bare.err;
    EXPECT_TRUE(contains(bare.err, "cube.stl: ")) << bare.err;
    EXPECT_FALSE(fs::exists(dir.path() / "bare"));
}

TEST(MeshPack, UsageErrorsExitTwoAndWriteNothing) {
    const spade::test::TempDir dir("mesh-pack-usage");
    const fs::path stl = dir.path() / "cube.stl";
    const fs::path mf = dir.path() / "cube.3mf";
    write_file(stl, stl_cube());
    write_file(mf, three_mf({cube_object(1, "hub")}));
    const fs::path out = dir.path() / "out";
    const struct {
        std::vector<fs::path> args;
        std::string needle;
    } cases[] = {
        {{}, "usage: spade_mesh_pack"},
        {{stl, "--unit", "mm"}, "--out"},
        {{stl, "--unit", "mm", "--out"}, "--out needs a value"},
        {{stl, "--unit", "mm", "--out", out, "--frobnicate"}, "--frobnicate"},
        {{stl, "--unit", "mm", "--out", out, "--out", out}, "--out is given twice"},
        {{stl, "--unit", "mm", "--out", out, "--tolerance", "0.1"}, "'0.1' has no unit"},
        {{stl, "--unit", "mm", "--out", out, "--tolerance", "-1mm"}, "'-1mm' is not finite and >= 0"},
        {{stl, "--unit", "furlong", "--out", out}, "'furlong' is not m, mm, cm, um, in or ft"},
        {{mf, "--unit", "mm", "--out", out}, "declares its own unit"},
        {{dir.path() / "part.step", "--out", out}, "'.step'"},
        {{stl, mf, "--out", out}, "one input"},
    };
    for (const auto& c : cases) {
        const auto run = spade::mesh_pack::run(c.args);
        EXPECT_EQ(run.exit_code, 2) << c.needle << ": " << run.err;
        EXPECT_TRUE(contains(run.err, c.needle)) << run.err;
        EXPECT_TRUE(contains(run.err, "usage: spade_mesh_pack")) << run.err;
        EXPECT_TRUE(run.out.empty()) << run.out;
        EXPECT_FALSE(fs::exists(out)) << c.needle;
    }
}

TEST(MeshPack, RefusalsExitByTheirCodeAndWriteNothing) {
    const spade::test::TempDir dir("mesh-pack-refused");
    write_file(dir.path() / "ascii.stl", "solid cube\n  facet normal 0 0 1\n");
    write_file(dir.path() / "open.obj", obj_cube(11));
    write_file(dir.path() / "ext.3mf",
               three_mf({cube_object(1, "hub")},
                        R"(unit="meter" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" )"
                        R"(requiredextensions="p")"));
    write_file(dir.path() / "a-file", "not a directory");
    const fs::path out = dir.path() / "out";
    const struct {
        std::vector<fs::path> args;
        int code;
        std::string needle;
    } cases[] = {
        {{dir.path() / "ascii.stl", "--unit", "mm", "--out", out}, 3, "export binary STL"},
        {{dir.path() / "ext.3mf", "--out", out}, 3, "'p'"},
        {{dir.path() / "open.obj", "--unit", "m", "--out", out}, 4, "object 'cube'"},
        {{dir.path() / "missing.stl", "--unit", "mm", "--out", out}, 1, "missing.stl: "},
        {{dir.path() / "ext.3mf", "--out", dir.path() / "a-file"}, 3, "'p'"},
    };
    for (const auto& c : cases) {
        const auto run = spade::mesh_pack::run(c.args);
        EXPECT_EQ(run.exit_code, c.code) << c.needle << ": " << run.err;
        EXPECT_TRUE(contains(run.err, c.needle)) << run.err;
        EXPECT_FALSE(fs::exists(out)) << c.needle;
    }

    // A good file whose --out cannot be made a directory: exit 1, named.
    write_file(dir.path() / "good.3mf", three_mf({cube_object(1, "hub")}));
    const auto blocked = pack({dir.path() / "good.3mf", "--out", dir.path() / "a-file"});
    EXPECT_EQ(blocked.exit_code, 1) << blocked.err;
    EXPECT_TRUE(contains(blocked.err, "a-file")) << blocked.err;
}

TEST(MeshPack, FileNamesAreSafeAndUniqueWithoutRegardToCase) {
    const spade::test::TempDir dir("mesh-pack-names");
    const fs::path in = dir.path() / "names.3mf";
    const char* const names[] = {"a/b", "A/B", "CON", "", "x. ", "lid", "nul.txt"};
    std::vector<std::string> objects;
    for (uint32_t i = 0; i < std::size(names); ++i) objects.push_back(cube_object(i + 1, names[i], 1.0, 2.0 * i));
    write_file(in, three_mf(objects, R"(unit="meter")"));
    const fs::path out = dir.path() / "out";
    const auto run = pack({in, "--out", out});
    ASSERT_EQ(run.exit_code, 0) << run.err;
    for (const char* const file :
         {"a_b.spmesh", "A_B-2.spmesh", "_CON.spmesh", "object.spmesh", "x.spmesh", "lid.spmesh", "_nul.txt.spmesh"}) {
        EXPECT_TRUE(fs::exists(out / file)) << file << "\n" << run.out;
    }
    std::size_t files = 0;
    for ([[maybe_unused]] const auto& entry : fs::directory_iterator(out)) ++files;
    EXPECT_EQ(files, std::size(names));
}

TEST(MeshPack, NamesThatDifferOnlyInUnicodeCaseNeverShareAFile) {
    // NTFS folds Unicode case ("a-umlaut" and "A-umlaut" are one name there),
    // which an ASCII-only fold cannot see: the second object would replace the
    // first. The write finds the target already there as a file written this
    // run, and bumps the suffix. On a case-sensitive file system the two names
    // are distinct from the start. Either way: two objects, two files.
    const spade::test::TempDir dir("mesh-pack-unicode-case");
    const fs::path in = dir.path() / "names.3mf";
    const std::string lower = "Geh\xC3\xA4" "use";  // Gehaeuse, a with umlaut
    const std::string upper = "GEH\xC3\x84" "USE";  // GEHAEUSE, A with umlaut
    write_file(in, three_mf({cube_object(1, lower), cube_object(2, upper, 1.0, 3.0)}, R"(unit="meter")"));
    const fs::path out = dir.path() / "out";
    const auto run = pack({in, "--out", out});
    ASSERT_EQ(run.exit_code, 0) << run.err;
    std::size_t files = 0;
    std::vector<fs::path> written;
    for (const auto& entry : fs::directory_iterator(out)) {
        ++files;
        written.push_back(entry.path());
    }
    EXPECT_EQ(files, 2u) << run.out;
    std::vector<uint64_t> hashes;
    for (const fs::path& path : written) {
        const auto loaded = spade::load_mesh_file(path);
        ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
        hashes.push_back(loaded->mesh->content_hash());
    }
    ASSERT_EQ(hashes.size(), 2u);
    EXPECT_NE(hashes[0], hashes[1]) << "the second object replaced the first";
}

TEST(MeshPack, ANegativeZeroToleranceIsRecordedAsPositiveZero) {
    const spade::test::TempDir dir("mesh-pack-negzero");
    const fs::path in = dir.path() / "cube.3mf";
    write_file(in, three_mf({cube_object(1, "hub")}));
    for (const char* const text : {"-0mm", "-0.0m", "0mm"}) {
        const auto run = pack({in, "--out", dir.path() / "out", "--tolerance", text});
        ASSERT_EQ(run.exit_code, 0) << text << ": " << run.err;
        const auto loaded = spade::load_mesh_file(dir.path() / "out" / "hub.spmesh");
        ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
        EXPECT_EQ(loaded->provenance.tolerance, 0.0) << text;
        EXPECT_FALSE(std::signbit(loaded->provenance.tolerance)) << text << " recorded -0.0";
    }
}
