// ---------------------------------------------------------------------------
// test_part_bounds.cpp -- sdf_bounds() and sdf_bounded() (world/sdf.hpp) and
// part_bounds() (vehicles/model_part.hpp): a part's box in its part frame, the
// lead's ask of 2026-10-13 for Physics' P2 grid.
//
// Exact on meshes. On SDF programs, CONSERVATIVE: every point eval() puts in
// the solid lies in the box (random CSG with turned, scaled leaves and smooth
// unions; points found by sampling and by bisecting to the surface), and
// TIGHT where the walk says so (axis-aligned union and intersect, a
// subtract's left operand, a smooth union's k/4), to the stated padding.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "vehicles/model_part.hpp"
#include "world/aabb.hpp"
#include "world/builder.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

namespace {

using spade::Aabb;
using spade::Code;
using spade::SdfNode;
using spade::SdfOp;
using spade::SdfPose;
using spade::SdfPrim;
using spade::SdfProgram;
using spade::vehicles::part_bounds;
using spade::vehicles::PartDesc;

// sdf_bounds()'s padding, per primitive under a rigid transform: 2^-16 of its
// largest coordinate (world/sdf.hpp, PADDING). A tight box is within two of
// those of the exact one, the second covering the outward fp32 rounding.
constexpr double kPad = 1.0 / 65536.0;

// Hand-made CSG in postfix order. Each leaf gets its own transform from
// transform_of(), the builder's own arithmetic.
struct Csg {
    SdfProgram program;

    Csg& leaf(SdfPrim kind, glm::vec4 params, const SdfPose& pose = {}) {
        SdfNode node{};
        node.kind = static_cast<uint32_t>(kind);
        node.op = static_cast<uint32_t>(SdfOp::none);
        node.params = params;
        node.transform = static_cast<uint32_t>(program.transforms.size());
        program.transforms.push_back(spade::transform_of(pose).value());
        program.nodes.push_back(node);
        return *this;
    }
    Csg& op(SdfOp which, float k = 0.0f) {
        SdfNode node{};
        node.op = static_cast<uint32_t>(which);
        node.params = glm::vec4(k, 0.0f, 0.0f, 0.0f);
        program.nodes.push_back(node);
        return *this;
    }
};

[[nodiscard]] SdfPose at(glm::vec3 position, glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                         float scale = 1.0f) {
    return SdfPose{.position = position, .rotation = rotation, .scale = scale};
}

[[nodiscard]] bool inside(const Aabb& box, glm::vec3 p) {
    return p.x >= box.min.x && p.y >= box.min.y && p.z >= box.min.z && p.x <= box.max.x && p.y <= box.max.y &&
           p.z <= box.max.z;
}

[[nodiscard]] std::string show(const Aabb& b) {
    return "[" + std::to_string(b.min.x) + ", " + std::to_string(b.min.y) + ", " + std::to_string(b.min.z) +
           "] .. [" + std::to_string(b.max.x) + ", " + std::to_string(b.max.y) + ", " + std::to_string(b.max.z) + "]";
}

// `box` holds [lo, hi] and exceeds it by no more than the padding of a
// primitive whose largest coordinate is `magnitude`.
void expect_tight(const Aabb& box, glm::dvec3 lo, glm::dvec3 hi, double magnitude, const std::string& what) {
    const double slack = 2.0 * kPad * magnitude;
    for (int i = 0; i < 3; ++i) {
        EXPECT_LE(box.min[i], lo[i]) << what << ": axis " << i << " min, " << show(box);
        EXPECT_GE(box.min[i], lo[i] - slack) << what << ": axis " << i << " min, " << show(box);
        EXPECT_GE(box.max[i], hi[i]) << what << ": axis " << i << " max, " << show(box);
        EXPECT_LE(box.max[i], hi[i] + slack) << what << ": axis " << i << " max, " << show(box);
    }
}

[[nodiscard]] Aabb bounds_of(const SdfProgram& program) {
    const spade::Result<Aabb> b = spade::sdf_bounds(program);
    EXPECT_TRUE(b.has_value()) << (b ? std::string() : b.error().context);
    return b.value_or(Aabb{});
}

[[nodiscard]] glm::dvec3 d(glm::vec3 v) {
    return glm::dvec3(v);
}

// --- one primitive at a time -------------------------------------------------

TEST(SdfBounds, EachPrimitiveIsBoundedByItsOwnBox) {
    const double r = 0.3f;
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::sphere, {0.3f, 0, 0, 0}).program), glm::dvec3(-r), glm::dvec3(r), r,
                 "sphere");

    const glm::dvec3 h = d({0.1f, 0.2f, 0.3f});
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::box, {0.1f, 0.2f, 0.3f, 0}).program), -h, h, h.z, "box");

    const glm::dvec3 cyl{double(0.15f), double(0.25f), double(0.15f)};
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::cylinder, {0.15f, 0.25f, 0, 0}).program), -cyl, cyl, cyl.y,
                 "cylinder");

    // A capsule's caps reach its radius past the half height.
    const glm::dvec3 cap{double(0.1f), double(0.2f) + double(0.1f), double(0.1f)};
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::capsule, {0.1f, 0.2f, 0, 0}).program), -cap, cap, cap.y, "capsule");

    // A torus: R + r across its ring (local XZ), r along its hole axis (+Y).
    const glm::dvec3 tor{double(0.4f) + double(0.05f), double(0.05f), double(0.4f) + double(0.05f)};
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::torus, {0.4f, 0.05f, 0, 0}).program), -tor, tor, tor.x, "torus");
}

TEST(SdfBounds, ATurnedLeafGetsTheBoxOfItsTurnedBox) {
    // A quarter turn about +Y swaps a box's x and z extents.
    const glm::quat quarter_y = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 1, 0));
    const glm::dvec3 swapped = d({0.3f, 0.2f, 0.1f});
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::box, {0.1f, 0.2f, 0.3f, 0}, at({0, 0, 0}, quarter_y)).program),
                 -swapped, swapped, 0.3, "box, a quarter turn about +Y");

    // An eighth turn about +Z, moved and scaled 2: a 0.1 m cube becomes a
    // 0.2 m cube whose x and y extents are its diagonal, 0.2 * sqrt(2).
    const glm::quat eighth_z = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));
    const glm::dvec3 centre = d({1.0f, -2.0f, 0.5f});
    const double diag = 0.2 * std::sqrt(2.0);
    const glm::dvec3 half{diag, diag, 0.2};
    expect_tight(
        bounds_of(Csg{}.leaf(SdfPrim::box, {0.1f, 0.1f, 0.1f, 0}, at({1.0f, -2.0f, 0.5f}, eighth_z, 2.0f)).program),
        centre - half, centre + half, 2.0 + diag, "a cube, turned an eighth about +Z, moved and scaled");

    // A capsule laid along +X (a quarter turn about +Z): its long axis is x.
    const glm::quat quarter_z = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1));
    const glm::dvec3 laid{double(0.2f) + double(0.1f), double(0.1f), double(0.1f)};
    expect_tight(bounds_of(Csg{}.leaf(SdfPrim::capsule, {0.1f, 0.2f, 0, 0}, at({0, 0, 0}, quarter_z)).program),
                 -laid, laid, laid.x, "a capsule laid along +X");
}

// --- the operators -----------------------------------------------------------

TEST(SdfBounds, UnionIntersectAndSubtractAreTightWhereTheyShouldBe) {
    // Two 1 m cubes, the second moved (0.6, 0.2, -0.1).
    const auto two_cubes = [](SdfOp which) {
        return Csg{}
            .leaf(SdfPrim::box, {0.5f, 0.5f, 0.5f, 0})
            .leaf(SdfPrim::box, {0.5f, 0.5f, 0.5f, 0}, at({0.6f, 0.2f, -0.1f}))
            .op(which)
            .program;
    };
    const glm::dvec3 a_lo(-0.5), a_hi(0.5);
    const glm::dvec3 b_lo = d({0.6f, 0.2f, -0.1f}) - 0.5, b_hi = d({0.6f, 0.2f, -0.1f}) + 0.5;
    expect_tight(bounds_of(two_cubes(SdfOp::union_)), glm::min(a_lo, b_lo), glm::max(a_hi, b_hi), 1.1, "union");
    expect_tight(bounds_of(two_cubes(SdfOp::intersect)), glm::max(a_lo, b_lo), glm::min(a_hi, b_hi), 1.1,
                 "intersect: exactly the overlap");
    expect_tight(bounds_of(two_cubes(SdfOp::subtract)), a_lo, a_hi, 1.1,
                 "subtract: the left operand's box, whatever the cut takes");

    // A tube, as the airframe compile makes one: a cylinder minus a thinner,
    // longer one. Its box is the outer cylinder's.
    const SdfProgram tube = Csg{}
                                .leaf(SdfPrim::cylinder, {0.02f, 0.1f, 0, 0})
                                .leaf(SdfPrim::cylinder, {0.015f, 0.11f, 0, 0})
                                .op(SdfOp::subtract)
                                .program;
    const glm::dvec3 outer{double(0.02f), double(0.1f), double(0.02f)};
    expect_tight(bounds_of(tube), -outer, outer, outer.y, "a tube");
}

TEST(SdfBounds, ASmoothUnionGrowsByAQuarterOfItsBlendRadius) {
    // Two 1 m spheres 1 m apart, blended with k = 2: the union's box grown by
    // k/4 = 0.5, the bound smooth_union's form guarantees.
    const auto blended = [](float k) {
        return Csg{}
            .leaf(SdfPrim::sphere, {1.0f, 0, 0, 0}, at({-0.5f, 0, 0}))
            .leaf(SdfPrim::sphere, {1.0f, 0, 0, 0}, at({0.5f, 0, 0}))
            .op(SdfOp::smooth_union, k)
            .program;
    };
    const glm::dvec3 union_lo(-1.5, -1.0, -1.0), union_hi(1.5, 1.0, 1.0);
    const Aabb grown = bounds_of(blended(2.0f));
    expect_tight(grown, union_lo - 0.5, union_hi + 0.5, 2.0, "k = 2");

    // The growth is real: the blend fills the waist past the spheres' own
    // y extent. (0, 1.4, 0) is in the solid, outside the plain union's box.
    const glm::vec3 waist(0.0f, 1.4f, 0.0f);
    EXPECT_LE(spade::eval(blended(2.0f), waist), 0.0f);
    EXPECT_GT(waist.y, union_hi.y);
    EXPECT_TRUE(inside(grown, waist)) << show(grown);

    // k = 0 is a plain union, and grows nothing.
    expect_tight(bounds_of(blended(0.0f)), union_lo, union_hi, 1.5, "k = 0");
}

// --- refusals ----------------------------------------------------------------

[[nodiscard]] std::string refusal(const SdfProgram& program) {
    const spade::Result<Aabb> b = spade::sdf_bounds(program);
    EXPECT_FALSE(b.has_value()) << "bounds " << (b ? show(*b) : std::string());
    if (b) return {};
    EXPECT_EQ(b.error().code, Code::invalid_argument) << b.error().context;
    return b.error().context;
}

TEST(SdfBounds, AnUnboundedSolidIsRefusedByName) {
    const glm::vec4 plane{0.0f, 1.0f, 0.0f, 0.2f};  // solid below y = 0.2
    const glm::vec4 sphere{0.5f, 0, 0, 0};
    const glm::vec4 hills{0.1f, 2.0f, 3.0f, 0.0f};

    const std::vector<std::pair<std::string, SdfProgram>> unbounded = {
        {"a plane", Csg{}.leaf(SdfPrim::plane, plane).program},
        {"a heightfield", Csg{}.leaf(SdfPrim::heightfield, hills).program},
        {"a sphere or a plane", Csg{}.leaf(SdfPrim::sphere, sphere).leaf(SdfPrim::plane, plane).op(SdfOp::union_).program},
        {"a plane blended with a sphere",
         Csg{}.leaf(SdfPrim::plane, plane).leaf(SdfPrim::sphere, sphere).op(SdfOp::smooth_union, 0.1f).program},
        {"a plane minus a sphere",
         Csg{}.leaf(SdfPrim::plane, plane).leaf(SdfPrim::sphere, sphere).op(SdfOp::subtract).program},
    };
    for (const auto& [what, program] : unbounded) {
        ASSERT_TRUE(program.validate().has_value()) << what;
        EXPECT_FALSE(spade::sdf_bounded(program)) << what;
        EXPECT_NE(refusal(program).find("unbounded"), std::string::npos) << what;
    }

    // An unbounded operand is fine where the walk bounds it: an intersect
    // takes the sphere's box (the plane does not clip it: conservative), and
    // "sphere minus plane" is the sphere's.
    const glm::dvec3 r(0.5);
    for (const SdfOp which : {SdfOp::intersect, SdfOp::subtract}) {
        const SdfProgram p = Csg{}.leaf(SdfPrim::sphere, sphere).leaf(SdfPrim::plane, plane).op(which).program;
        EXPECT_TRUE(spade::sdf_bounded(p));
        expect_tight(bounds_of(p), -r, r, 0.5, "a sphere with a plane");
    }
    const SdfProgram flipped = Csg{}.leaf(SdfPrim::plane, plane).leaf(SdfPrim::sphere, sphere).op(SdfOp::intersect).program;
    EXPECT_TRUE(spade::sdf_bounded(flipped));
    expect_tight(bounds_of(flipped), -r, r, 0.5, "a plane with a sphere");
}

TEST(SdfBounds, AnEmptySolidIsRefusedByName) {
    // Two 0.1 m spheres 1 m apart have no point in common.
    const SdfProgram apart = Csg{}
                                 .leaf(SdfPrim::sphere, {0.1f, 0, 0, 0})
                                 .leaf(SdfPrim::sphere, {0.1f, 0, 0, 0}, at({1.0f, 0, 0}))
                                 .op(SdfOp::intersect)
                                 .program;
    EXPECT_TRUE(spade::sdf_bounded(apart)) << "bounded by its structure; empty by its numbers";
    EXPECT_NE(refusal(apart).find("empty"), std::string::npos);

    // An empty program has no solid either.
    EXPECT_FALSE(spade::sdf_bounded(SdfProgram{}));
    EXPECT_NE(refusal(SdfProgram{}).find("empty"), std::string::npos);
}

TEST(SdfBounds, ASingularTransformIsRefusedByName) {
    // A hand-built transform that flattens y: validate() takes it (finite, a
    // positive scale), but the sphere's solid is an infinite slab along y.
    SdfProgram p = Csg{}.leaf(SdfPrim::sphere, {0.5f, 0, 0, 0}).program;
    p.transforms[0].world_to_local[1][1] = 0.0f;
    ASSERT_TRUE(p.validate().has_value());
    EXPECT_TRUE(spade::sdf_bounded(p)) << "the structure reads no transform";
    const std::string why = refusal(p);
    EXPECT_NE(why.find("singular"), std::string::npos) << why;
    EXPECT_NE(why.find("node 0"), std::string::npos) << why;
}

TEST(SdfBounds, WhatValidateRefusesPassesThrough) {
    const SdfProgram p = Csg{}.leaf(SdfPrim::sphere, {-0.5f, 0, 0, 0}).program;
    const spade::Result<Aabb> b = spade::sdf_bounds(p);
    ASSERT_FALSE(b.has_value());
    EXPECT_EQ(b.error(), p.validate().error());
}

// --- containment, on random CSG ------------------------------------------------

// Random programs: up to eight leaves (every bounded kind), each turned by a
// random rotation, moved and scaled; union, intersect, subtract and
// smooth_union (k up to 0.6) between them. From a fixed stream, so the same
// programs on every platform.
struct RandomCsg {
    spade::rng::Stream s{.state = 0x9a7b0c1d2e3fULL, .cached_gauss = 0.0f, .has_cached = 0u};

    float uniform(float lo, float hi) { return lo + (hi - lo) * s.next_float(); }

    void leaf(Csg& c) {
        const uint64_t pick = s.next_u64() % 5;
        const glm::quat turn(uniform(-1, 1), uniform(-1, 1), uniform(-1, 1), uniform(-1, 1));
        const SdfPose pose = at({uniform(-0.8f, 0.8f), uniform(-0.8f, 0.8f), uniform(-0.8f, 0.8f)}, turn,
                                uniform(0.5f, 1.5f));
        switch (pick) {
            case 0: c.leaf(SdfPrim::sphere, {uniform(0.05f, 0.5f), 0, 0, 0}, pose); break;
            case 1: c.leaf(SdfPrim::box, {uniform(0.05f, 0.5f), uniform(0.05f, 0.5f), uniform(0.05f, 0.5f), 0}, pose); break;
            case 2: c.leaf(SdfPrim::cylinder, {uniform(0.05f, 0.4f), uniform(0.05f, 0.5f), 0, 0}, pose); break;
            case 3: c.leaf(SdfPrim::capsule, {uniform(0.05f, 0.3f), uniform(0.05f, 0.5f), 0, 0}, pose); break;
            default: c.leaf(SdfPrim::torus, {uniform(0.1f, 0.5f), uniform(0.02f, 0.2f), 0, 0}, pose); break;
        }
    }

    void tree(Csg& c, int depth) {
        if (depth == 0 || s.next_float() < 0.25f) {
            leaf(c);
            return;
        }
        tree(c, depth - 1);
        tree(c, depth - 1);
        switch (s.next_u64() % 4) {
            case 0: c.op(SdfOp::union_); break;
            case 1: c.op(SdfOp::intersect); break;
            case 2: c.op(SdfOp::subtract); break;
            default: c.op(SdfOp::smooth_union, uniform(0.0f, 0.6f)); break;
        }
    }
};

TEST(SdfBounds, EveryPointInsideRandomCsgLiesInItsBox) {
    RandomCsg gen;
    int bounded = 0;
    int with_points = 0;
    int empty = 0;
    for (int program_index = 0; program_index < 60; ++program_index) {
        Csg c;
        gen.tree(c, 3);
        const SdfProgram& p = c.program;
        ASSERT_TRUE(p.validate().has_value()) << "program " << program_index;
        ASSERT_TRUE(spade::sdf_bounded(p)) << "program " << program_index;
        const spade::Result<Aabb> b = spade::sdf_bounds(p);

        // Every leaf is within 0.8 m of the origin and at most about 1.1 m
        // across, so +-2.5 m holds every solid.
        const auto anywhere = [&] { return glm::vec3(gen.uniform(-2.5f, 2.5f), gen.uniform(-2.5f, 2.5f), gen.uniform(-2.5f, 2.5f)); };

        if (!b) {
            // The only refusal a random program of bounded leaves can earn:
            // proved empty. Then nothing may evaluate inside.
            ASSERT_NE(b.error().context.find("empty"), std::string::npos) << b.error().context;
            ++empty;
            for (int i = 0; i < 2000; ++i) {
                const glm::vec3 q = anywhere();
                ASSERT_GT(spade::eval(p, q), 0.0f) << "program " << program_index << " was refused as empty";
            }
            continue;
        }
        ++bounded;
        const Aabb box = *b;
        ASSERT_TRUE(box.min.x <= box.max.x && box.min.y <= box.max.y && box.min.z <= box.max.z) << show(box);

        // Points in the box grown by a quarter of its size each way, then
        // points anywhere: each one the solid holds lies in the box.
        const glm::vec3 margin = 0.25f * (box.max - box.min) + 0.01f;
        std::vector<glm::vec3> found;
        for (int i = 0; i < 2000; ++i) {
            const glm::vec3 q =
                i < 1000 ? glm::vec3(gen.uniform(box.min.x - margin.x, box.max.x + margin.x),
                                     gen.uniform(box.min.y - margin.y, box.max.y + margin.y),
                                     gen.uniform(box.min.z - margin.z, box.max.z + margin.z))
                         : anywhere();
            if (spade::eval(p, q) <= 0.0f) {
                ASSERT_TRUE(inside(box, q)) << "program " << program_index << ", point (" << q.x << ", " << q.y
                                            << ", " << q.z << ") is in the solid, outside " << show(box);
                found.push_back(q);
            }
        }
        if (!found.empty()) ++with_points;

        // The surface: from up to 40 inside points, walk out along a random
        // direction to a point past the grown box, and bisect to the surface,
        // keeping the inside end.
        const float reach = 2.0f * glm::length(box.max - box.min) + 1.0f;
        for (size_t i = 0; i < found.size() && i < 40; ++i) {
            const glm::vec3 dir = glm::normalize(glm::vec3(gen.uniform(-1, 1), gen.uniform(-1, 1), gen.uniform(-1, 1)) +
                                                 glm::vec3(1e-3f, 0.0f, 0.0f));
            glm::vec3 in = found[i];
            glm::vec3 out = in + reach * dir;
            if (spade::eval(p, out) <= 0.0f) continue;
            for (int step = 0; step < 40; ++step) {
                const glm::vec3 mid = 0.5f * (in + out);
                (spade::eval(p, mid) <= 0.0f ? in : out) = mid;
            }
            ASSERT_TRUE(inside(box, in)) << "program " << program_index << ", surface point (" << in.x << ", "
                                         << in.y << ", " << in.z << ") outside " << show(box);
        }
    }
    // Not vacuous: most programs have a solid, and sampling finds it.
    EXPECT_GE(bounded, 40) << "empty: " << empty;
    EXPECT_GE(with_points, 30);
}

// --- parts ---------------------------------------------------------------------

// A box's twelve triangles, counter-clockwise seen from outside, its corners
// numbered x + 2y + 4z (tests/test_mesh_field.cpp's cube).
constexpr uint32_t kBoxTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // -z
    {4, 5, 6}, {5, 7, 6},  // +z
    {0, 1, 4}, {1, 5, 4},  // -y
    {2, 6, 3}, {3, 6, 7},  // +y
    {0, 4, 2}, {2, 4, 6},  // -x
    {1, 3, 5}, {3, 7, 5},  // +x
};

[[nodiscard]] std::shared_ptr<const spade::TriangleMesh> box_mesh(glm::dvec3 centre, glm::dvec3 size,
                                                                   spade::MeshUnit unit) {
    std::vector<glm::dvec3> positions;
    for (uint32_t v = 0; v < 8; ++v) {
        const glm::dvec3 corner{(v & 1u) != 0 ? 0.5 : -0.5, ((v >> 1) & 1u) != 0 ? 0.5 : -0.5,
                                ((v >> 2) & 1u) != 0 ? 0.5 : -0.5};
        positions.push_back(centre + size * corner);
    }
    std::vector<uint32_t> indices;
    for (const auto& t : kBoxTriangles) indices.insert(indices.end(), {t[0], t[1], t[2]});
    const spade::MeshInput input{.positions = std::move(positions), .indices = std::move(indices), .unit = unit};
    return spade::make_mesh(input, "box").value();
}

TEST(PartBounds, AMeshPartsBoxIsItsVerticesExactly) {
    // Off-centre and in millimetres: make_mesh() stored fp32 metres, and the
    // box is exactly their least and greatest.
    PartDesc part{};
    part.name = "hull";
    part.mesh = box_mesh(glm::dvec3(20.0, -10.0, 5.0), glm::dvec3(300.0, 100.0, 200.0), spade::MeshUnit::millimetre);
    glm::vec3 lo = part.mesh->positions()[0], hi = lo;
    for (const glm::vec3& v : part.mesh->positions()) {
        lo = glm::min(lo, v);
        hi = glm::max(hi, v);
    }
    const spade::Result<Aabb> b = part_bounds(part);
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_EQ(b->min, lo);
    EXPECT_EQ(b->max, hi);
    EXPECT_EQ(b->min, glm::vec3(float(-130.0 * 0.001), float(-60.0 * 0.001), float(-95.0 * 0.001)));
    EXPECT_EQ(b->max, glm::vec3(float(170.0 * 0.001), float(40.0 * 0.001), float(105.0 * 0.001)));
}

TEST(PartBounds, AnSdfPartsBoxIsItsShapesBox) {
    PartDesc part{};
    part.name = "arm";
    part.shape = Csg{}
                     .leaf(SdfPrim::cylinder, {0.01f, 0.12f, 0, 0},
                           at({0.1f, 0, 0}, glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1))))
                     .leaf(SdfPrim::sphere, {0.02f, 0, 0, 0}, at({0.22f, 0, 0}))
                     .op(SdfOp::union_)
                     .program;
    const spade::Result<Aabb> b = part_bounds(part);
    const spade::Result<Aabb> want = spade::sdf_bounds(part.shape);
    ASSERT_TRUE(b.has_value()) << b.error().context;
    ASSERT_TRUE(want.has_value()) << want.error().context;
    EXPECT_EQ(b->min, want->min);
    EXPECT_EQ(b->max, want->max);
}

TEST(PartBounds, APartWithBothFormsOrNeitherIsRefusedByName) {
    PartDesc neither{};
    neither.name = "ghost";
    const spade::Result<Aabb> none = part_bounds(neither);
    ASSERT_FALSE(none.has_value());
    EXPECT_NE(none.error().context.find("part 'ghost'"), std::string::npos) << none.error().context;
    EXPECT_NE(none.error().context.find("neither"), std::string::npos) << none.error().context;

    PartDesc both{};
    both.name = "twin";
    both.shape = Csg{}.leaf(SdfPrim::sphere, {0.1f, 0, 0, 0}).program;
    both.mesh = box_mesh(glm::dvec3(0.0), glm::dvec3(0.1), spade::MeshUnit::metre);
    const spade::Result<Aabb> two = part_bounds(both);
    ASSERT_FALSE(two.has_value());
    EXPECT_NE(two.error().context.find("part 'twin'"), std::string::npos) << two.error().context;
    EXPECT_NE(two.error().context.find("both"), std::string::npos) << two.error().context;

    // A shape's own refusal names the part too.
    PartDesc open{};
    open.name = "floor";
    open.shape = Csg{}.leaf(SdfPrim::plane, {0.0f, 1.0f, 0.0f, 0.0f}).program;
    const spade::Result<Aabb> unbounded = part_bounds(open);
    ASSERT_FALSE(unbounded.has_value());
    EXPECT_NE(unbounded.error().context.find("part 'floor'"), std::string::npos) << unbounded.error().context;
    EXPECT_NE(unbounded.error().context.find("unbounded"), std::string::npos) << unbounded.error().context;
}

}  // namespace
