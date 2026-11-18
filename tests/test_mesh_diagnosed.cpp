#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "world/mesh.hpp"

// ---------------------------------------------------------------------------
// world/mesh.hpp: make_mesh_diagnosed() and MeshRefusal (L88, Interface's stable
// gate kinds). make_mesh_diagnosed() is make_mesh() that also says WHICH refusal
// it gave; make_mesh() is it with the kind dropped, so the Error (code and message)
// of every refusal is the same bytes from both. One kind per distinct refusal site of
// make_mesh(), in the order it reaches them; refusal_name() spells the kind and is
// the stable `code` of spade_part_import's gate refusal, so every spelling is
// pinned here byte for byte.
//
// The fixtures are hand-built (a cube whose vertices are numbered x + 2y + 4z, a
// tetrahedron) so each one trips exactly the refusal it is named for.
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::MeshInput;
using spade::MeshRefusal;
using spade::MeshUnit;

constexpr uint32_t kCubeTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
    {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5},
};

MeshInput cube(glm::dvec3 lo, double side, MeshUnit unit = MeshUnit::metre) {
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

MeshInput unit_cube() {
    return cube({0.0, 0.0, 0.0}, 1.0);
}

MeshInput flipped(MeshInput m) {
    for (std::size_t t = 0; t < m.indices.size(); t += 3) {
        std::swap(m.indices[t + 1], m.indices[t + 2]);
    }
    return m;
}

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
    for (const uint32_t i : b.indices) {
        m.indices.push_back(remap[i]);
    }
    return m;
}

MeshInput tetrahedron() {
    MeshInput m;
    m.unit = MeshUnit::metre;
    m.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};  // counter-clockwise seen from outside
    return m;
}

// One fixture per kind that a hand-built input can reach. too_large (more than 2^31
// triangles) cannot be built in a test; its spelling is pinned below.
struct Fixture {
    MeshRefusal kind;
    const char* name;  // refusal_name(kind), spelt out independently of the code
    MeshInput input;
};

std::vector<Fixture> fixtures() {
    MeshInput unit_unspecified = unit_cube();
    unit_unspecified.unit = MeshUnit::unspecified;

    MeshInput unit_invalid = unit_cube();
    unit_invalid.unit = static_cast<MeshUnit>(99);

    MeshInput bad_index_count = unit_cube();
    bad_index_count.indices.pop_back();

    MeshInput out_of_range = unit_cube();
    out_of_range.indices[5] = 8;

    MeshInput nan = unit_cube();
    nan.positions[3].y = std::numeric_limits<double>::quiet_NaN();

    MeshInput overflows = unit_cube();
    overflows.positions[7] = {1e300, 1e300, 1e300};

    MeshInput unreferenced = unit_cube();
    unreferenced.positions.push_back({5.0, 5.0, 5.0});

    MeshInput coincident = unit_cube();
    coincident.positions.push_back({0.0, 0.0, 0.0});
    coincident.indices[0] = 8;

    MeshInput repeated = unit_cube();
    repeated.indices[1] = 0;

    MeshInput collinear = unit_cube();
    collinear.positions.push_back({0.5, 0.0, 0.0});
    collinear.indices.insert(collinear.indices.end(), {0u, 8u, 1u});

    MeshInput duplicate = unit_cube();
    duplicate.indices.insert(duplicate.indices.end(), {kCubeTriangles[2][0], kCubeTriangles[2][1], kCubeTriangles[2][2]});

    MeshInput open = unit_cube();
    open.indices.erase(open.indices.begin() + 33, open.indices.begin() + 36);

    MeshInput misoriented = unit_cube();
    std::swap(misoriented.indices[13], misoriented.indices[14]);

    MeshInput pushed = unit_cube();
    pushed.positions[7] = {0.3, 0.4, -0.6};

    MeshInput crowd;
    crowd.unit = MeshUnit::metre;
    const MeshInput tet = tetrahedron();
    for (uint32_t k = 0; k <= spade::kMaxMeshShells; ++k) {
        const uint32_t base = static_cast<uint32_t>(crowd.positions.size());
        for (const glm::dvec3& p : tet.positions) {
            crowd.positions.push_back(p + glm::dvec3{2.0 * k, 0.0, 0.0});
        }
        for (const uint32_t i : tet.indices) {
            crowd.indices.push_back(base + i);
        }
    }

    return {
        {MeshRefusal::unit_unspecified, "unit_unspecified", unit_unspecified},
        {MeshRefusal::unit_invalid, "unit_invalid", unit_invalid},
        {MeshRefusal::empty, "empty", MeshInput{{}, {}, MeshUnit::metre}},
        {MeshRefusal::bad_index_count, "bad_index_count", bad_index_count},
        {MeshRefusal::index_out_of_range, "index_out_of_range", out_of_range},
        {MeshRefusal::non_finite, "non_finite", nan},
        {MeshRefusal::non_finite_scaled, "non_finite_scaled", overflows},
        {MeshRefusal::unreferenced_vertex, "unreferenced_vertex", unreferenced},
        {MeshRefusal::coincident_vertices, "coincident_vertices", coincident},
        {MeshRefusal::repeated_index, "repeated_index", repeated},
        {MeshRefusal::degenerate_triangle, "degenerate_triangle", collinear},
        {MeshRefusal::duplicate_triangle, "duplicate_triangle", duplicate},
        {MeshRefusal::not_closed, "not_closed", open},
        {MeshRefusal::not_manifold, "not_manifold", merged(unit_cube(), cube({1.0, 1.0, 0.0}, 1.0))},
        {MeshRefusal::inconsistent_winding, "inconsistent_winding", misoriented},
        {MeshRefusal::not_manifold_vertex, "not_manifold_vertex", merged(unit_cube(), cube({1.0, 1.0, 1.0}, 1.0))},
        {MeshRefusal::inside_out, "inside_out", flipped(unit_cube())},
        {MeshRefusal::too_many_shells, "too_many_shells", crowd},
        {MeshRefusal::self_intersecting, "self_intersecting", pushed},
        {MeshRefusal::shells_not_nested, "shells_not_nested", merged(unit_cube(), cube({0.25, 0.25, 0.25}, 0.5))},
    };
}

}  // namespace

TEST(MeshDiagnosed, EachRefusalSiteNamesItsKindAndKeepsMakeMeshsErrorBytes) {
    for (const Fixture& f : fixtures()) {
        SCOPED_TRACE(f.name);
        const auto diagnosed = spade::make_mesh_diagnosed(f.input, "fixture");
        ASSERT_FALSE(diagnosed.has_value());
        EXPECT_EQ(diagnosed.error().kind, f.kind);
        EXPECT_STREQ(spade::refusal_name(diagnosed.error().kind), f.name);
        // make_mesh() is the same call with the kind dropped: the same Error, byte for byte.
        const auto plain = spade::make_mesh(f.input, "fixture");
        ASSERT_FALSE(plain.has_value());
        EXPECT_EQ(plain.error().code, diagnosed.error().error.code);
        EXPECT_EQ(plain.error().context, diagnosed.error().error.context);
        EXPECT_EQ(diagnosed.error().error.context.rfind("fixture: ", 0), 0u) << diagnosed.error().error.context;
    }
}

TEST(MeshDiagnosed, TooManyShellsIsCapacityExceededAndEveryOtherBuildableKindIsInvalidArgument) {
    for (const Fixture& f : fixtures()) {
        SCOPED_TRACE(f.name);
        const auto diagnosed = spade::make_mesh_diagnosed(f.input, "fixture");
        ASSERT_FALSE(diagnosed.has_value());
        EXPECT_EQ(diagnosed.error().error.code,
                  f.kind == MeshRefusal::too_many_shells ? Code::capacity_exceeded : Code::invalid_argument);
    }
}

TEST(MeshDiagnosed, EveryKindIsSpeltByteForByteAndOnce) {
    // The order is the order make_mesh() reaches the refusals in. These strings are another tool's
    // stable codes: a spelling that changes is a break.
    const char* const expected[] = {
        "unit_unspecified", "unit_invalid",       "empty",              "bad_index_count",     "too_large",
        "index_out_of_range", "non_finite",       "non_finite_scaled",  "unreferenced_vertex", "coincident_vertices",
        "repeated_index",   "degenerate_triangle", "duplicate_triangle", "not_closed",          "not_manifold",
        "inconsistent_winding", "not_manifold_vertex", "inside_out",     "too_many_shells",     "self_intersecting",
        "shells_not_nested",
    };
    const MeshRefusal kinds[] = {
        MeshRefusal::unit_unspecified,   MeshRefusal::unit_invalid,       MeshRefusal::empty,
        MeshRefusal::bad_index_count,    MeshRefusal::too_large,          MeshRefusal::index_out_of_range,
        MeshRefusal::non_finite,         MeshRefusal::non_finite_scaled,  MeshRefusal::unreferenced_vertex,
        MeshRefusal::coincident_vertices, MeshRefusal::repeated_index,    MeshRefusal::degenerate_triangle,
        MeshRefusal::duplicate_triangle, MeshRefusal::not_closed,         MeshRefusal::not_manifold,
        MeshRefusal::inconsistent_winding, MeshRefusal::not_manifold_vertex, MeshRefusal::inside_out,
        MeshRefusal::too_many_shells,    MeshRefusal::self_intersecting,  MeshRefusal::shells_not_nested,
    };
    static_assert(sizeof(expected) / sizeof(expected[0]) == 21);
    static_assert(sizeof(kinds) / sizeof(kinds[0]) == 21);
    std::set<std::string> seen;
    for (std::size_t i = 0; i < 21; ++i) {
        EXPECT_STREQ(spade::refusal_name(kinds[i]), expected[i]) << i;
        EXPECT_TRUE(seen.insert(spade::refusal_name(kinds[i])).second) << expected[i];
    }
    // The values are consecutive from 0, in that order (a tool may index a table by them).
    for (std::size_t i = 0; i < 21; ++i) {
        EXPECT_EQ(static_cast<std::size_t>(kinds[i]), i) << expected[i];
    }
}

TEST(MeshDiagnosed, AnAcceptedMeshIsTheSameMeshMakeMeshBuilds) {
    const MeshInput in = cube({-0.5, 2.0, 0.25}, 0.75);
    const auto diagnosed = spade::make_mesh_diagnosed(in, "cube");
    ASSERT_TRUE(diagnosed.has_value()) << diagnosed.error().error.context;
    const auto plain = spade::make_mesh(in, "cube");
    ASSERT_TRUE(plain.has_value());
    EXPECT_EQ((*diagnosed)->content_hash(), (*plain)->content_hash());
    EXPECT_EQ((*diagnosed)->triangle_count(), 12u);
    EXPECT_EQ((*diagnosed)->shell_count(), 1u);
}
