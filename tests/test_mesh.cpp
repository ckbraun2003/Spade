#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "world/mesh.hpp"

// ---------------------------------------------------------------------------
// world/mesh.hpp: the mesh type, its units and its validation (the parts and
// meshes plan, Task B1; CORE-6).
//
// Every fixture is built by hand from a cube whose vertices are numbered
// x + 2y + 4z, so each expected count and element below can be read off the
// fixture. Where two cubes share a position, the fixture welds exact
// duplicates itself, as a reader would, so the refusal under test is the one
// that fires and not "weld first".
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::MeshInput;
using spade::MeshUnit;
using spade::TriangleMesh;

// The cube's twelve triangles, counter-clockwise seen from outside.
constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // z = lo, normal -z
    {4, 5, 6}, {5, 7, 6},  // z = hi, normal +z
    {0, 1, 4}, {1, 5, 4},  // y = lo, normal -y
    {2, 6, 3}, {3, 6, 7},  // y = hi, normal +y
    {0, 4, 2}, {2, 4, 6},  // x = lo, normal -x
    {1, 3, 5}, {3, 7, 5},  // x = hi, normal +x
};

MeshInput cube(glm::dvec3 lo, double side, MeshUnit unit) {
    MeshInput m;
    m.unit = unit;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{static_cast<double>(v & 1u), static_cast<double>((v >> 1) & 1u),
                                static_cast<double>((v >> 2) & 1u)};
        m.positions.push_back(lo + side * corner);
    }
    for (const auto& t : kCubeTriangles) {
        m.indices.insert(m.indices.end(), {t[0], t[1], t[2]});
    }
    return m;
}

MeshInput unit_cube(MeshUnit unit) {
    return cube({0.0, 0.0, 0.0}, 1.0, unit);
}

// `a` then `b` as one input, welding b's exact duplicates of a's positions.
MeshInput merged(const MeshInput& a, const MeshInput& b) {
    MeshInput m = a;
    std::vector<uint32_t> remap;
    for (const glm::dvec3& p : b.positions) {
        const auto it = std::find(m.positions.begin(), m.positions.end(), p);
        if (it == m.positions.end()) {
            remap.push_back(static_cast<uint32_t>(m.positions.size()));
            m.positions.push_back(p);
        } else {
            remap.push_back(static_cast<uint32_t>(it - m.positions.begin()));
        }
    }
    for (const uint32_t i : b.indices) m.indices.push_back(remap[i]);
    return m;
}

MeshInput cube_without_triangle(uint32_t t) {
    MeshInput m = unit_cube(MeshUnit::metre);
    m.indices.erase(m.indices.begin() + 3 * t, m.indices.begin() + 3 * t + 3);
    return m;
}

MeshInput cube_with_flipped_triangle(uint32_t t) {
    MeshInput m = unit_cube(MeshUnit::metre);
    std::swap(m.indices[3 * t + 1], m.indices[3 * t + 2]);
    return m;
}

MeshInput cubes_sharing_edge() {  // the second cube's edge x = y = 1 is the first's
    return merged(unit_cube(MeshUnit::metre), cube({1.0, 1.0, 0.0}, 1.0, MeshUnit::metre));
}

MeshInput cubes_sharing_vertex() {  // only (1, 1, 1) is shared
    return merged(unit_cube(MeshUnit::metre), cube({1.0, 1.0, 1.0}, 1.0, MeshUnit::metre));
}

MeshInput cube_with_duplicate_triangle(uint32_t t) {
    MeshInput m = unit_cube(MeshUnit::metre);
    m.indices.insert(m.indices.end(), {kCubeTriangles[t][0], kCubeTriangles[t][1], kCubeTriangles[t][2]});
    return m;
}

MeshInput cube_with_reversed_duplicate_triangle(uint32_t t) {
    MeshInput m = unit_cube(MeshUnit::metre);
    m.indices.insert(m.indices.end(), {kCubeTriangles[t][0], kCubeTriangles[t][2], kCubeTriangles[t][1]});
    return m;
}

MeshInput inside_out_cube() {
    MeshInput m = unit_cube(MeshUnit::metre);
    for (std::size_t t = 0; t < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
    return m;
}

// The bytes Decisions spell the content hash over, written out independently
// of the implementation (TD-4): the vertex count, the triangle count, each
// position's float bits (x, y, z), then each index, all u32 little-endian.
std::vector<uint8_t> unit_cube_bytes() {
    std::vector<uint8_t> out;
    const auto u32 = [&out](uint32_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    const MeshInput m = unit_cube(MeshUnit::metre);
    u32(8);
    u32(12);
    for (const glm::dvec3& p : m.positions) {
        u32(std::bit_cast<uint32_t>(static_cast<float>(p.x)));
        u32(std::bit_cast<uint32_t>(static_cast<float>(p.y)));
        u32(std::bit_cast<uint32_t>(static_cast<float>(p.z)));
    }
    for (const uint32_t i : m.indices) u32(i);
    return out;
}

uint64_t reference_fnv(const std::vector<uint8_t>& bytes) {
    uint64_t h = 0xcbf29ce484222325ULL;
    for (const uint8_t b : bytes) {
        h ^= b;
        h *= 0x100000001b3ULL;
    }
    return h;
}

struct Case {
    const char* why;
    MeshInput input;
    const char* needle;
};

void expect_refused(const Case& c) {
    const auto m = spade::make_mesh(c.input, "fixture");
    ASSERT_FALSE(m.has_value()) << c.why;
    EXPECT_EQ(m.error().code, Code::invalid_argument) << c.why;
    EXPECT_NE(m.error().context.find(c.needle), std::string::npos) << c.why << ": " << m.error().context;
    EXPECT_EQ(m.error().context.rfind("fixture: ", 0), 0u) << c.why << ": " << m.error().context;
}

}  // namespace

TEST(Mesh, EachTopologyRefusalNamesItsCountAndFirstElements) {
    const Case cases[] = {
        {"one face removed", cube_without_triangle(11), "3 open edges"},
        {"one face flipped", cube_with_flipped_triangle(4), "3 misoriented edges"},
        {"two cubes sharing an edge", cubes_sharing_edge(), "1 non-manifold edge"},
        {"two cubes touching at a corner", cubes_sharing_vertex(), "1 non-manifold vertex"},
        {"a triangle twice", cube_with_duplicate_triangle(2), "1 duplicate triangle"},
        {"a triangle twice, once reversed", cube_with_reversed_duplicate_triangle(2), "1 duplicate triangle"},
        {"every face flipped", inside_out_cube(), "inside out"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.why);
        expect_refused(c);
    }

    // The first elements, in (lower index, higher index) order: triangle 11 is
    // (3, 7, 5), so its three edges are the open ones.
    const auto open = spade::make_mesh(cube_without_triangle(11), "fixture");
    ASSERT_FALSE(open.has_value());
    EXPECT_EQ(open.error().context, "fixture: 3 open edges, first (3, 5) (3, 7) (5, 7)");
}

TEST(Mesh, TheContentHashIsSpeltByteByByte) {
    // An independent FNV-1a 64 over the spelling Decisions gives (TD-4): the counts,
    // each position's float bits and each index, little-endian.
    const auto m = spade::make_mesh(unit_cube(MeshUnit::metre), "cube");
    ASSERT_TRUE(m.has_value()) << m.error().context;
    EXPECT_EQ((*m)->content_hash(), reference_fnv(unit_cube_bytes()));
}

TEST(Mesh, AClosedCubeIsAccepted) {
    const MeshInput in = cube({-0.5, 2.0, 0.25}, 0.75, MeshUnit::metre);
    const auto m = spade::make_mesh(in, "cube");
    ASSERT_TRUE(m.has_value()) << m.error().context;
    const TriangleMesh& mesh = **m;
    EXPECT_EQ(mesh.triangle_count(), 12u);
    EXPECT_EQ(mesh.shell_count(), 1u);
    ASSERT_EQ(mesh.positions().size(), 8u);
    for (std::size_t v = 0; v < 8; ++v) {
        EXPECT_EQ(mesh.positions()[v], glm::vec3(in.positions[v])) << "vertex " << v;
    }
    ASSERT_EQ(mesh.indices().size(), 36u);
    EXPECT_TRUE(std::equal(mesh.indices().begin(), mesh.indices().end(), in.indices.begin()));
}

TEST(Mesh, DegenerateAndInvalidInputIsRefused) {
    MeshInput not_triples = unit_cube(MeshUnit::metre);
    not_triples.indices.pop_back();

    MeshInput out_of_range = unit_cube(MeshUnit::metre);
    out_of_range.indices[5] = 8;

    MeshInput nan = unit_cube(MeshUnit::metre);
    nan.positions[3].y = std::numeric_limits<double>::quiet_NaN();

    MeshInput overflows = unit_cube(MeshUnit::metre);  // finite in double, not in fp32
    overflows.positions[7] = {1e300, 1e300, 1e300};

    MeshInput unreferenced = unit_cube(MeshUnit::metre);
    unreferenced.positions.push_back({5.0, 5.0, 5.0});

    MeshInput coincident = unit_cube(MeshUnit::metre);  // vertex 8 sits on vertex 0
    coincident.positions.push_back({0.0, 0.0, 0.0});
    coincident.indices[0] = 8;

    MeshInput repeated = unit_cube(MeshUnit::metre);  // triangle 0 becomes (0, 0, 1)
    repeated.indices[1] = 0;

    MeshInput collinear = unit_cube(MeshUnit::metre);  // (0, 8, 1) lies on the edge (0, 1)
    collinear.positions.push_back({0.5, 0.0, 0.0});
    collinear.indices.insert(collinear.indices.end(), {0u, 8u, 1u});

    const Case cases[] = {
        {"an empty input", MeshInput{{}, {}, MeshUnit::metre}, "no triangles"},
        {"an index count that is not a multiple of 3", not_triples, "not a multiple of 3"},
        {"an out-of-range index", out_of_range, "1 index out of range"},
        {"a NaN", nan, "1 position not finite"},
        {"a position past fp32's range", overflows, "1 position not finite after scaling"},
        {"an unreferenced vertex", unreferenced, "1 unreferenced vertex"},
        {"coincident vertices", coincident, "weld first"},
        {"a repeated index", repeated, "1 triangle repeats an index"},
        {"three collinear vertices", collinear, "1 degenerate triangle"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.why);
        expect_refused(c);
    }
}

TEST(Mesh, UnitsAreDeclaredAndConvertedOnce) {
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::unspecified), 0.0);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::metre), 1.0);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::millimetre), 0.001);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::centimetre), 0.01);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::micrometre), 1e-6);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::inch), 0.0254);
    EXPECT_EQ(spade::mesh_unit_scale(MeshUnit::foot), 0.3048);
    EXPECT_EQ(spade::mesh_unit_scale(static_cast<MeshUnit>(99)), 0.0);

    expect_refused({"an unspecified unit", unit_cube(MeshUnit::unspecified), "unit is unspecified"});

    // Scaled in double, rounded to fp32 once.
    const auto mm = spade::make_mesh(cube({0.0, 0.0, 0.0}, 10.0, MeshUnit::millimetre), "mm");
    ASSERT_TRUE(mm.has_value()) << mm.error().context;
    ASSERT_EQ((*mm)->positions().size(), 8u);
    const float ten_mm = static_cast<float>(10.0 * 0.001);
    EXPECT_EQ((*mm)->positions()[0], glm::vec3(0.0f));
    EXPECT_EQ((*mm)->positions()[7], glm::vec3(ten_mm));
    EXPECT_EQ((*mm)->positions()[5], glm::vec3(ten_mm, 0.0f, ten_mm));

    const auto in = spade::make_mesh(unit_cube(MeshUnit::inch), "inch");
    ASSERT_TRUE(in.has_value()) << in.error().context;
    ASSERT_EQ((*in)->positions().size(), 8u);
    EXPECT_EQ((*in)->positions()[7], glm::vec3(static_cast<float>(1.0 * 0.0254)));

    const auto ft = spade::make_mesh(unit_cube(MeshUnit::foot), "foot");
    ASSERT_TRUE(ft.has_value()) << ft.error().context;
    ASSERT_EQ((*ft)->positions().size(), 8u);
    EXPECT_EQ((*ft)->positions()[7], glm::vec3(static_cast<float>(1.0 * 0.3048)));
}

TEST(Mesh, TwoShellsAreCounted) {
    // Two disjoint cubes: accepted, two shells. Whether one may sit inside the
    // other is Task B2's nesting check.
    const auto m = spade::make_mesh(
        merged(unit_cube(MeshUnit::metre), cube({3.0, 0.0, 0.0}, 1.0, MeshUnit::metre)), "two");
    ASSERT_TRUE(m.has_value()) << m.error().context;
    EXPECT_EQ((*m)->triangle_count(), 24u);
    EXPECT_EQ((*m)->shell_count(), 2u);
}
