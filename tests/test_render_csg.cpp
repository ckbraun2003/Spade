#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <iomanip>
#include <optional>
#include <sstream>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "render/csg_mesh.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

// ---------------------------------------------------------------------------
// CSG subtree meshing tests -- S7a Task R5.
//
// Three burdens of proof, same shape as test_render_tessellate.cpp's:
//
//   * split_program() partitions a REAL postfix program correctly -- verified
//     against the exact three shapes the task brief names: a pure-union
//     program (hover-pad: plane, cylinder, union -- 2 primitives, 0 CSG
//     roots), a pure-subtract program (the gate-square prefab: box, box,
//     subtract -- 0 primitives, 1 CSG root spanning the whole program), and
//     tests/golden/worlds/maximal.world.yaml -- a COMMITTED golden world
//     already exercising intersect and smooth_union, loaded for real rather
//     than re-authored by hand here (so this test cannot silently drift from
//     what the corpus actually contains).
//
//   * mesh_csg_subtree() produces a mesh with a GENUINE HOLE -- the whole
//     point of this task (task brief: "the hole you fly through"). Proven by
//     ray-casting, not by a vertex/index count: a ray along the gate axis
//     through the centre must find ZERO intersections; a ray offset into the
//     frame body must find exactly TWO (front face in, back face out).
//
//   * TRIANGLE WINDING is consistently outward -- surface nets faces the
//     identical hazard tessellate.cpp's file comment documents (four of
//     seven primitives wound backward, invisible to every check except this
//     one), and DrawMode::shaded now culls back faces (ruling SR-13), so a
//     backward triangle here would render as an invisible hole in the mesh
//     rather than a geometry defect anyone could see in a vertex dump.
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::Code;
using spade::Result;
using spade::SdfPose;
using spade::SdfProgram;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::Aabb;
using spade::render::CsgMeshLimits;
using spade::render::csg_subtree_world_bounds;
using spade::render::kCsgMeshDefaults;
using spade::render::mesh_csg_subtree;
using spade::render::MeshData;
using spade::render::split_program;
using spade::render::SubtreeSplit;

// ===========================================================================
// A tiny, self-contained SHA-256 (FIPS 180-4) -- test-only, identical
// construction to test_render_tessellate.cpp's and test_render_raster.cpp's
// own copies (each a private, internal-linkage helper in its own
// translation unit; there is nothing to share a header for). Verified below
// against the two textbook vectors before this file's own golden test trusts
// it, under a suite name ("CsgMeshSha256") distinct from the other two
// files' own self-tests -- GoogleTest requires a unique (suite, case) pair
// per binary.
// ===========================================================================

constexpr std::array<uint32_t, 64> kSha256RoundConstants = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

[[nodiscard]] constexpr uint32_t rotr32(uint32_t x, uint32_t n) noexcept { return (x >> n) | (x << (32u - n)); }

void sha256_process_block(std::array<uint32_t, 8>& h, const uint8_t block[64]) {
    std::array<uint32_t, 64> w{};
    for (uint32_t i = 0; i < 16; ++i) {
        w[i] = (static_cast<uint32_t>(block[4 * i]) << 24) | (static_cast<uint32_t>(block[4 * i + 1]) << 16) |
               (static_cast<uint32_t>(block[4 * i + 2]) << 8) | static_cast<uint32_t>(block[4 * i + 3]);
    }
    for (uint32_t i = 16; i < 64; ++i) {
        const uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (uint32_t i = 0; i < 64; ++i) {
        const uint32_t s1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
        const uint32_t ch = (e & f) ^ ((~e) & g);
        const uint32_t temp1 = hh + s1 + ch + kSha256RoundConstants[i] + w[i];
        const uint32_t s0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
        const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const uint32_t temp2 = s0 + maj;
        hh = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
}

[[nodiscard]] std::string sha256_hex(std::span<const std::byte> data) {
    std::array<uint32_t, 8> h = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                  0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

    const uint64_t bit_length = static_cast<uint64_t>(data.size()) * 8u;
    std::vector<uint8_t> message(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        message[i] = std::to_integer<uint8_t>(data[i]);
    }
    message.push_back(0x80u);
    while (message.size() % 64u != 56u) {
        message.push_back(0u);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<uint8_t>((bit_length >> shift) & 0xffu));
    }

    for (size_t offset = 0; offset < message.size(); offset += 64u) {
        sha256_process_block(h, &message[offset]);
    }

    std::ostringstream out;
    for (const uint32_t word : h) {
        out << std::hex << std::setw(8) << std::setfill('0') << word;
    }
    return out.str();
}

template <class T>
[[nodiscard]] std::span<const std::byte> as_bytes_of(const std::vector<T>& v) {
    return std::as_bytes(std::span<const T>(v));
}

// ===========================================================================
// Fixtures
// ===========================================================================

WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-csg-test").capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    return b;
}

WorldDesc build_or_fail(const WorldBuilder& b) {
    const Result<WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "builder failed: " << world.error().context;
        return WorldDesc{};
    }
    return *world;
}

// hover-pad's own shape (task brief): plane, cylinder, union. A pure-union
// program -- every leaf tessellates individually, no CSG root anywhere.
WorldDesc hover_pad_world() {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).cylinder(0.5f, 0.1f).union_();
    return build_or_fail(b);
}

// The gate-square prefab's own shape (task brief): box, box, subtract.
// Outer half-extents (2, 2, 0.3) -- a square slab; inner half-extents
// (1.2, 1.2, 0.5) -- narrower in X/Y (leaving a 0.8-unit frame border) and
// DEEPER in Z than the outer slab's own half-depth, so the subtracted box
// punches all the way through rather than leaving a blind pocket. Both at
// the identity pose, so the hole is centred on the gate's own local origin
// and its axis is world +Z -- "the hole you fly through" (task brief).
WorldDesc gate_square_world() {
    WorldBuilder b = base_builder();
    b.box(glm::vec3(2.0f, 2.0f, 0.3f)).box(glm::vec3(1.2f, 1.2f, 0.5f)).subtract();
    return build_or_fail(b);
}

[[nodiscard]] std::filesystem::path golden_world_path(const char* name) {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "worlds" / name;
}

Result<SubtreeSplit> split_or_fail(const SdfProgram& program) {
    Result<SubtreeSplit> split = split_program(program);
    if (!split) {
        ADD_FAILURE() << "split_program failed: " << split.error().context;
    }
    return split;
}

Aabb bounds_or_fail(const SdfProgram& program, uint32_t root, const Aabb& fallback = Aabb{}) {
    const Result<Aabb> bounds = csg_subtree_world_bounds(program, root, fallback);
    if (!bounds) {
        ADD_FAILURE() << "csg_subtree_world_bounds failed: " << bounds.error().context;
        return Aabb{};
    }
    return *bounds;
}

MeshData mesh_or_fail(const SdfProgram& program, uint32_t root, const Aabb& bounds,
                       const CsgMeshLimits& limits = kCsgMeshDefaults) {
    const Result<MeshData> mesh = mesh_csg_subtree(program, root, bounds, limits);
    if (!mesh) {
        ADD_FAILURE() << "mesh_csg_subtree failed: " << mesh.error().context;
        return MeshData{};
    }
    return *mesh;
}

// ===========================================================================
// A tiny ray/triangle intersector (Moller-Trumbore) -- test-only, exactly
// enough to count crossings for the hole test below. Not a rendering path;
// this file's only use for it is a topological probe.
// ===========================================================================

[[nodiscard]] std::optional<float> ray_triangle_hit(glm::vec3 origin, glm::vec3 dir, glm::vec3 v0, glm::vec3 v1,
                                                     glm::vec3 v2) {
    constexpr float kEpsilon = 1e-7f;
    const glm::vec3 edge1 = v1 - v0;
    const glm::vec3 edge2 = v2 - v0;
    const glm::vec3 pvec = glm::cross(dir, edge2);
    const float det = glm::dot(edge1, pvec);
    if (std::fabs(det) < kEpsilon) {
        return std::nullopt;  // ray parallel to the triangle's plane
    }
    const float inv_det = 1.0f / det;
    const glm::vec3 tvec = origin - v0;
    const float u = glm::dot(tvec, pvec) * inv_det;
    if (u < 0.0f || u > 1.0f) {
        return std::nullopt;
    }
    const glm::vec3 qvec = glm::cross(tvec, edge1);
    const float v = glm::dot(dir, qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f) {
        return std::nullopt;
    }
    const float t = glm::dot(edge2, qvec) * inv_det;
    if (t < kEpsilon) {
        return std::nullopt;  // behind the ray origin
    }
    return t;
}

[[nodiscard]] int count_ray_mesh_intersections(const MeshData& mesh, glm::vec3 origin, glm::vec3 dir, float max_t) {
    int count = 0;
    for (size_t t = 0; t + 3 <= mesh.indices.size(); t += 3) {
        const glm::vec3& v0 = mesh.positions[mesh.indices[t]];
        const glm::vec3& v1 = mesh.positions[mesh.indices[t + 1]];
        const glm::vec3& v2 = mesh.positions[mesh.indices[t + 2]];
        const std::optional<float> hit = ray_triangle_hit(origin, dir, v0, v1, v2);
        if (hit.has_value() && *hit < max_t) {
            ++count;
        }
    }
    return count;
}

}  // namespace

// ===========================================================================
// 0. The golden manifest's own hash function is trustworthy.
// ===========================================================================

TEST(CsgMeshSha256, MatchesTheStandardTestVectors) {
    const std::string empty;
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(empty.data(), empty.size()))),
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    const std::string abc = "abc";
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(abc.data(), abc.size()))),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

// ===========================================================================
// 1. split_program() -- the three shapes the task brief names.
// ===========================================================================

TEST(SplitProgram, HoverPadFindsTwoUnionPrimitivesNoCsgRoots) {
    const WorldDesc world = hover_pad_world();
    const Result<SubtreeSplit> split = split_or_fail(world.sdf);
    ASSERT_TRUE(split.has_value());

    EXPECT_EQ(split->union_primitive_nodes.size(), 2u);
    EXPECT_TRUE(split->csg_roots.empty());
    // plane (node 0), cylinder (node 1) -- program's own authoring order.
    EXPECT_EQ(split->union_primitive_nodes, (std::vector<uint32_t>{0u, 1u}));
}

TEST(SplitProgram, GateSquarePrefabFindsOneCsgRootNoPrimitives) {
    const WorldDesc world = gate_square_world();
    const Result<SubtreeSplit> split = split_or_fail(world.sdf);
    ASSERT_TRUE(split.has_value());

    EXPECT_TRUE(split->union_primitive_nodes.empty());
    ASSERT_EQ(split->csg_roots.size(), 1u);
    // subtract is node 2 (box, box, subtract) -- the whole 3-node program is
    // one CSG root.
    EXPECT_EQ(split->csg_roots[0], 2u);
}

TEST(SplitProgram, GoldenWorldWithIntersectAndSmoothUnionFindsExpectedRootsAndPrimitives) {
    // tests/golden/worlds/maximal.world.yaml -- a COMMITTED golden world,
    // loaded for real (not re-authored here) so this test tracks whatever
    // the corpus actually contains. Its program (that file's own `sdf.nodes`
    // list):
    //   0 plane, 1 sphere, 2 union, 3 box, 4 intersect, 5 cylinder,
    //   6 capsule, 7 subtract, 8 smooth_union, 9 torus, 10 heightfield,
    //   11 union, 12 union (the program's own root).
    // Node 12 (union) decomposes into node 8 (smooth_union -- a CSG root
    // whose own subtree spans [0, 8], i.e. (plane u sphere) n box,
    // smooth-blended with (cylinder - capsule)) and node 11 (union, which
    // decomposes further into torus/heightfield, two ordinary primitives).
    const Result<WorldDesc> world = spade::load_world_file(golden_world_path("maximal.world.yaml"));
    ASSERT_TRUE(world.has_value()) << "failed to load maximal.world.yaml";

    const Result<SubtreeSplit> split = split_or_fail(world->sdf);
    ASSERT_TRUE(split.has_value());

    EXPECT_EQ(split->csg_roots, (std::vector<uint32_t>{8u}));
    EXPECT_EQ(split->union_primitive_nodes, (std::vector<uint32_t>{9u, 10u}));
}

TEST(SplitProgram, EmptyProgramSplitsToNothing) {
    const SdfProgram empty{};
    const Result<SubtreeSplit> split = split_or_fail(empty);
    ASSERT_TRUE(split.has_value());
    EXPECT_TRUE(split->union_primitive_nodes.empty());
    EXPECT_TRUE(split->csg_roots.empty());
}

TEST(SplitProgram, SingleUnwrappedPrimitiveIsAUnionPrimitiveNotACsgRoot) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const WorldDesc world = build_or_fail(b);

    const Result<SubtreeSplit> split = split_or_fail(world.sdf);
    ASSERT_TRUE(split.has_value());
    EXPECT_EQ(split->union_primitive_nodes, (std::vector<uint32_t>{0u}));
    EXPECT_TRUE(split->csg_roots.empty());
}

// ===========================================================================
// 2. csg_subtree_world_bounds() -- CSG-AWARE bounds per operator (S7a Task
//    R5 fix wave, review IMPORTANT #1): each operator narrows or discards
//    an operand's extent according to what it actually DOES to the solid
//    region, not a flat union of every leaf's own shape regardless of the
//    operator sitting above it. Getting this wrong does not corrupt
//    geometry (the derived bound is always a SUPERSET of the true solid) --
//    it starves resolution, silently and without bound, which is why each
//    case below is pinned to an exact expected Aabb rather than just
//    "non-empty".
// ===========================================================================

TEST(CsgSubtreeWorldBounds, SubtractUsesOnlyTheMinuendsBoundsNotTheSubtrahends) {
    // gate_square_world(): outer half=(2,2,0.3) "a" (minuend), inner
    // half=(1.2,1.2,0.5) "b" (subtrahend, the standard oversized-cutter
    // idiom -- deeper than the slab it cuts through). "a minus b" is a
    // SUBSET of a alone (sdf.cpp: max(a,-b)) -- the subtrahend's own extent,
    // even though it happens to be larger on Z, must never widen the bound.
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2, Aabb{});

    constexpr float kTol = 1e-5f;
    EXPECT_NEAR(bounds.min.x, -2.0f, kTol);
    EXPECT_NEAR(bounds.min.y, -2.0f, kTol);
    EXPECT_NEAR(bounds.min.z, -0.3f, kTol) << "the subtrahend's larger Z extent (0.5) leaked into the bound";
    EXPECT_NEAR(bounds.max.x, 2.0f, kTol);
    EXPECT_NEAR(bounds.max.y, 2.0f, kTol);
    EXPECT_NEAR(bounds.max.z, 0.3f, kTol) << "the subtrahend's larger Z extent (0.5) leaked into the bound";
}

TEST(CsgSubtreeWorldBounds, IntersectWithAnUnboundedOperandUsesTheOtherOperandsTightBoundNotTheFallback) {
    // plane has no finite local extent (PA-5) -- but "a b intersect" is a
    // SUBSET of BOTH a and b (sdf.cpp: max(a,b)), so intersecting with an
    // unbounded operand cannot make the OTHER, genuinely bounded operand's
    // own extent any looser. The correct bound is the box's own tight
    // extent, NOT a world-sized fallback (the pre-fix behaviour) and not
    // the plane's own fallback at all.
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).box(glm::vec3(1.0f)).intersect();
    const WorldDesc world = build_or_fail(b);

    const Aabb fallback{.min = glm::vec3(-9.0f), .max = glm::vec3(9.0f)};
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2, fallback);

    EXPECT_EQ(bounds.min, glm::vec3(-1.0f));
    EXPECT_EQ(bounds.max, glm::vec3(1.0f));
    EXPECT_NE(bounds.min, fallback.min) << "fell back to the world-sized bounds instead of the box's own tight one";
}

TEST(CsgSubtreeWorldBounds, IntersectOfTwoBoundedOperandsIsTheirComponentwiseIntersection) {
    // Two boxes, same origin, each tight on a DIFFERENT pair of axes: A is
    // tight on X (half=1) and wide on Y/Z (half=3); B is tight on Y
    // (half=1) and wide on X/Z (half=3). Their true intersection is tight
    // on BOTH X and Y (half=1) and wide only on Z (half=3) -- a genuine
    // per-axis clamp, not just "the smaller of the two boxes as a whole".
    WorldBuilder b = base_builder();
    b.box(glm::vec3(1.0f, 3.0f, 3.0f)).box(glm::vec3(3.0f, 1.0f, 3.0f)).intersect();
    const WorldDesc world = build_or_fail(b);

    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2, Aabb{});

    EXPECT_EQ(bounds.min, glm::vec3(-1.0f, -1.0f, -3.0f));
    EXPECT_EQ(bounds.max, glm::vec3(1.0f, 1.0f, 3.0f));
}

TEST(CsgSubtreeWorldBounds, UnboundedMinuendStillFallsBackWhenNothingElseBoundsIt) {
    // "a minus b" uses a's bound alone (the test above) -- when a is ITSELF
    // unbounded (a plane), the whole subtract is still unbounded regardless
    // of what b is, so this is the one remaining case the fallback exists
    // for: the subtree's OWN extent stays unbounded after every operator in
    // it has had its say.
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).box(glm::vec3(1.0f)).subtract();
    const WorldDesc world = build_or_fail(b);

    const Aabb fallback{.min = glm::vec3(-9.0f), .max = glm::vec3(9.0f)};
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2, fallback);

    EXPECT_EQ(bounds.min, fallback.min);
    EXPECT_EQ(bounds.max, fallback.max);
}

TEST(CsgSubtreeWorldBounds, SmoothUnionDilatesTheUnionBoundByKOverFour) {
    // cylinder (radius=1, half_height=1 -> box (-1,-1,-1)..(1,1,1)) and a
    // sphere (radius=1) offset to (3,0,0) (-> (2,-1,-1)..(4,1,1)), blended
    // with k=0.4. sdf.cpp's own bound on the blend (min(a,b) - k/4 <= d <=
    // min(a,b)) means the true surface can sit up to k/4 = 0.1 beyond the
    // NAIVE union's own boundary -- the bound must include that margin, not
    // just union_aabb(a,b) verbatim.
    WorldBuilder b = base_builder();
    b.cylinder(1.0f, 1.0f).sphere(1.0f, SdfPose{.position = {3.0f, 0.0f, 0.0f}}).smooth_union(0.4f);
    const WorldDesc world = build_or_fail(b);

    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2, Aabb{});

    constexpr float kTol = 1e-5f;
    const glm::vec3 expected_min(-1.1f, -1.1f, -1.1f);
    const glm::vec3 expected_max(4.1f, 1.1f, 1.1f);
    EXPECT_NEAR(bounds.min.x, expected_min.x, kTol);
    EXPECT_NEAR(bounds.min.y, expected_min.y, kTol);
    EXPECT_NEAR(bounds.min.z, expected_min.z, kTol);
    EXPECT_NEAR(bounds.max.x, expected_max.x, kTol);
    EXPECT_NEAR(bounds.max.y, expected_max.y, kTol);
    EXPECT_NEAR(bounds.max.z, expected_max.z, kTol);
}

TEST(CsgSubtreeWorldBounds, RootNodeOutOfRangeIsAnError) {
    const WorldDesc world = gate_square_world();
    const Result<Aabb> bounds = csg_subtree_world_bounds(world.sdf, /*root=*/99, Aabb{});
    ASSERT_FALSE(bounds.has_value());
    EXPECT_EQ(bounds.error().code, Code::invalid_argument);
}

// ===========================================================================
// 3. mesh_csg_subtree() -- the hole test. This is the point of the task.
// ===========================================================================

TEST(MeshCsgSubtree, GateAxisRayFindsNoIntersectionsFrameBodyRayFindsTwo) {
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const MeshData mesh = mesh_or_fail(world.sdf, /*root=*/2, bounds);

    ASSERT_FALSE(mesh.positions.empty()) << "surface nets produced no geometry at all";
    ASSERT_FALSE(mesh.indices.empty());

    // Through the hole: (x, y) well inside the inner (subtracted) box's
    // (1.2, 1.2) footprint, offset slightly off-centre so the ray does not
    // graze the sampling grid's own lines exactly on axis. The inner box's
    // half-depth (0.5) exceeds the outer slab's (0.3), so the subtraction
    // punches all the way through -- zero surface crossings anywhere along
    // this line.
    const glm::vec3 hole_origin(0.05f, 0.07f, -10.0f);
    const glm::vec3 axis(0.0f, 0.0f, 1.0f);
    EXPECT_EQ(count_ray_mesh_intersections(mesh, hole_origin, axis, 20.0f), 0)
        << "a ray through the gate's centre hit the mesh -- the hole is missing (rendered as a solid "
           "block, exactly the defect this task exists to prevent)";

    // Through the frame body: x = 1.6 is inside the outer box's (2.0) half-
    // extent but OUTSIDE the inner box's (1.2) half-extent, so this column
    // is solid, untouched frame material for the outer slab's full 0.3
    // half-depth -- front face in, back face out, exactly two crossings.
    const glm::vec3 frame_origin(1.6f, 0.07f, -10.0f);
    EXPECT_EQ(count_ray_mesh_intersections(mesh, frame_origin, axis, 20.0f), 2)
        << "a ray through solid frame material did not find exactly two crossings (front + back face)";
}

// The stated winding convention (tessellate.cpp's own file comment,
// "CCW as seen from outside") applies here too, and DrawMode::shaded now
// culls back faces (SR-13) -- so a backward triangle renders as an invisible
// gap, not a visible defect a screenshot would catch. For every
// non-degenerate triangle, cross(p1-p0, p2-p0) must point the same way as
// the vertices' own stored (gradient-derived) normals, summed rather than
// averaged since only the SIGN of the dot product matters.
TEST(MeshCsgSubtree, TriangleWindingIsConsistentlyOutward) {
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const MeshData mesh = mesh_or_fail(world.sdf, /*root=*/2, bounds);

    ASSERT_EQ(mesh.indices.size() % 3u, 0u);
    ASSERT_FALSE(mesh.indices.empty());

    constexpr float kDegenerateAreaEpsilon = 1e-9f;
    size_t non_degenerate_triangles = 0;
    for (size_t t = 0; t + 3 <= mesh.indices.size(); t += 3) {
        const uint32_t ia = mesh.indices[t];
        const uint32_t ib = mesh.indices[t + 1];
        const uint32_t ic = mesh.indices[t + 2];
        const glm::vec3& pa = mesh.positions[ia];
        const glm::vec3& pb = mesh.positions[ib];
        const glm::vec3& pc = mesh.positions[ic];

        const glm::vec3 face_normal = glm::cross(pb - pa, pc - pa);
        if (glm::length(face_normal) < kDegenerateAreaEpsilon) {
            continue;
        }
        ++non_degenerate_triangles;

        const glm::vec3 stored_normal_sum = mesh.normals[ia] + mesh.normals[ib] + mesh.normals[ic];
        EXPECT_GT(glm::dot(face_normal, stored_normal_sum), 0.0f)
            << "triangle " << (t / 3) << " is wound backward -- its face normal points opposite its own "
               "vertices' stored (gradient) normals";
    }
    EXPECT_GT(non_degenerate_triangles, 0u) << "every triangle was degenerate -- this test checked nothing";
}

TEST(MeshCsgSubtree, ProducesFiniteGeometryWithValidIndicesAndEmptySubmeshArrays) {
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const MeshData mesh = mesh_or_fail(world.sdf, /*root=*/2, bounds);

    ASSERT_EQ(mesh.positions.size(), mesh.normals.size());
    for (const uint32_t index : mesh.indices) {
        ASSERT_LT(index, mesh.positions.size());
    }
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
        const glm::vec3& p = mesh.positions[i];
        EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) << "vertex " << i;
        const glm::vec3& n = mesh.normals[i];
        EXPECT_TRUE(std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z)) << "vertex " << i;
        EXPECT_NEAR(glm::length(n), 1.0f, 1e-3f) << "vertex " << i << ": normal not unit-length";
    }

    // SR-11: single-material, empty submesh triple, exactly
    // tessellate_primitive()'s own producer contract.
    EXPECT_TRUE(mesh.submesh_first_index.empty());
    EXPECT_TRUE(mesh.submesh_index_count.empty());
    EXPECT_TRUE(mesh.submesh_material.empty());
}

TEST(MeshCsgSubtree, RootNodeOutOfRangeIsAnError) {
    const WorldDesc world = gate_square_world();
    const Result<MeshData> mesh = mesh_csg_subtree(world.sdf, /*root=*/99, Aabb{});
    ASSERT_FALSE(mesh.has_value());
    EXPECT_EQ(mesh.error().code, Code::invalid_argument);
}

TEST(MeshCsgSubtree, LimitsThatGiveNoCellsAreAnError) {
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const CsgMeshLimits no_cell_size{.cell_size = 0.0f};
    const CsgMeshLimits no_floor{.min_cells_per_axis = 0};
    const CsgMeshLimits cap_below_floor{.min_cells_per_axis = 48, .max_cells_per_axis = 8};
    for (const CsgMeshLimits& bad : {no_cell_size, no_floor, cap_below_floor}) {
        const Result<MeshData> mesh = mesh_csg_subtree(world.sdf, /*root=*/2, bounds, bad);
        ASSERT_FALSE(mesh.has_value());
        EXPECT_EQ(mesh.error().code, Code::invalid_argument);
    }
}

// B1 (rendering/plans/2026-10-03-raster-defects-plan.md): the grid follows a
// world-space cell, so a large subtree is meshed as finely as a small one.
// A small subtree keeps the old 48 cells; a large one is capped.
TEST(CsgCellsPerAxis, TheGridFollowsAWorldSpaceCellSize) {
    const CsgMeshLimits limits{.cell_size = 0.05f, .min_cells_per_axis = 48, .max_cells_per_axis = 160,
                               .aabb_margin = 0.05f};
    const auto cube = [](float half) { return Aabb{.min = glm::vec3(-half), .max = glm::vec3(half)}; };
    EXPECT_EQ(spade::render::csg_cells_per_axis(cube(0.5f), limits), 48u) << "1.1 m keeps the old floor";
    EXPECT_EQ(spade::render::csg_cells_per_axis(cube(2.0f), limits), 82u) << "4.1 m / 0.05 m";
    EXPECT_EQ(spade::render::csg_cells_per_axis(cube(4.0f), limits), 160u) << "8.1 m reaches the cap";
    const Aabb slab{.min = glm::vec3(-2.0f, -0.1f, -0.1f), .max = glm::vec3(2.0f, 0.1f, 0.1f)};
    EXPECT_EQ(spade::render::csg_cells_per_axis(slab, limits), 82u) << "the longest axis decides";
}

// B3: a wall about one cell thick folds under surface nets and draws with
// holes. shower's bowl (sphere r 4 minus sphere r 3.8 at y 0.4) thins to
// nothing at its rim, so some fold at any cell size, and scene_from_world()
// must name it in a warning (L6), never build it in silence.
TEST(CsgFolds, AThinShellFoldsAndTheSceneNamesIt) {
    WorldBuilder b = base_builder();
    b.sphere(4.0f).sphere(3.8f, SdfPose{.position = {0.0f, 0.4f, 0.0f}}).subtract();
    const WorldDesc world = build_or_fail(b);
    const Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene) << scene.error().context;
    ASSERT_EQ(scene->warnings.size(), 1u) << "one CSG subtree, so one warning";
    const std::string& warning = scene->warnings[0];
    EXPECT_NE(warning.find("node 2"), std::string::npos) << warning;
    EXPECT_NE(warning.find("thinner than about two cells"), std::string::npos) << warning;

    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const MeshData mesh = mesh_or_fail(world.sdf, /*root=*/2, bounds);
    const spade::render::CsgFoldReport folds = spade::render::find_folded_triangles(
        world.sdf, 2, mesh, spade::render::csg_mesh_cell_size(world.sdf, 2, bounds));
    EXPECT_GT(folds.folded, 0u);
    EXPECT_GT(folds.bounds.min.y, 0.0f) << "the folds are in the thin upper wall, not the thick bottom";
    EXPECT_EQ(folds.thin, folds.folded) << "every fold here is on the thin wall";
    EXPECT_EQ(folds.sharp, 0u);

    // Since B2, shaded and velocity frames march the wall and draw it whole
    // (RS3). Only the mesh has the holes, and the mesh draws wireframe and
    // casts the sun's shadow, so the warning says that and no more.
    EXPECT_NE(warning.find("wireframe"), std::string::npos) << warning;
    EXPECT_NE(warning.find("shadow"), std::string::npos) << warning;
    EXPECT_NE(warning.find("draw it whole"), std::string::npos) << warning;
}

// The warning's cell is the cell the mesh samples. A smooth_union root pads
// its sample box by k/4 when that beats aabb_margin (csg_subtree_sample_box()),
// so a figure from aabb_margin alone names a cell the mesh does not use.
TEST(CsgFolds, TheWarningNamesTheCellTheMeshSamples) {
    WorldBuilder b = base_builder();
    b.sphere(4.0f).sphere(3.8f, SdfPose{.position = {0.0f, 0.4f, 0.0f}}).subtract();
    b.sphere(0.5f, SdfPose{.position = {0.0f, -4.2f, 0.0f}}).smooth_union(2.0f);  // k/4 = 0.5 m
    const WorldDesc world = build_or_fail(b);
    const Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene) << scene.error().context;
    ASSERT_EQ(scene->warnings.size(), 1u) << "the bowl's rim still folds";
    const std::string& warning = scene->warnings[0];

    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/4);
    const float cell = spade::render::csg_mesh_cell_size(world.sdf, 4, bounds);
    const glm::vec3 extent = bounds.max - bounds.min + glm::vec3(2.0f * kCsgMeshDefaults.aabb_margin);
    const float margin_only = std::max({extent.x, extent.y, extent.z}) /
                              static_cast<float>(spade::render::csg_cells_per_axis(bounds, kCsgMeshDefaults));
    ASSERT_NE(std::format("{:.3f}", cell), std::format("{:.3f}", margin_only))
        << "the case must tell the two figures apart";
    EXPECT_NE(warning.find(std::format("two cells of {:.3f} m", cell)), std::string::npos)
        << "the mesh samples " << cell << " m cells: " << warning;
}

// A sharp CSG edge folds a few triangles at any thickness: the gradient
// flips across the crease. Kat's circuit-track node 19 is such a shape, a
// torus with a 0.24 m tube cut flat by a box at y = 1.6. Its tube is about
// five cells thick, so "make it thicker" would be the wrong advice. The
// warning names a sharp edge instead, and says it is expected.
TEST(CsgFolds, ASharpEdgeIsNamedAsOneNotAsAThinWall) {
    WorldBuilder b = base_builder();
    b.torus(2.0f, 0.24f, SdfPose{.position = {0.0f, 1.5f, 0.0f}});
    b.box(glm::vec3(5.0f, 5.0f, 5.0f), SdfPose{.position = {0.0f, 1.6f - 5.0f, 0.0f}});
    b.intersect();
    const WorldDesc world = build_or_fail(b);

    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);
    const MeshData mesh = mesh_or_fail(world.sdf, /*root=*/2, bounds);
    const spade::render::CsgFoldReport folds = spade::render::find_folded_triangles(
        world.sdf, 2, mesh, spade::render::csg_mesh_cell_size(world.sdf, 2, bounds));
    ASSERT_GT(folds.folded, 0u) << "the case must fold at its crease";
    EXPECT_EQ(folds.sharp, folds.folded) << "every fold here is at the sharp cut edge";
    EXPECT_EQ(folds.thin, 0u);
    EXPECT_NEAR(folds.sharp_bounds.max.y, 1.6f, 0.1f) << "the folds lie along the cut";

    const Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene) << scene.error().context;
    ASSERT_EQ(scene->warnings.size(), 1u);
    const std::string& warning = scene->warnings[0];
    EXPECT_NE(warning.find("sharp edge"), std::string::npos) << warning;
    EXPECT_NE(warning.find("smooth_union"), std::string::npos) << warning;
    EXPECT_EQ(warning.find("thinner than"), std::string::npos) << "not a thin wall: " << warning;
}

// The control: the same bowl with a thick wall folds nowhere and warns of
// nothing, so the warning above is about thinness, not about subtract.
TEST(CsgFolds, AThickShellNeitherFoldsNorWarns) {
    WorldBuilder b = base_builder();
    b.sphere(4.0f).sphere(3.0f, SdfPose{.position = {0.0f, 0.4f, 0.0f}}).subtract();
    const WorldDesc world = build_or_fail(b);
    const Result<spade::render::RenderScene> scene = spade::render::scene_from_world(world, {});
    ASSERT_TRUE(scene) << scene.error().context;
    EXPECT_TRUE(scene->warnings.empty()) << scene->warnings[0];
}

// ===========================================================================
// 4. Determinism: identical inputs, byte-identical output (constraint 3).
// ===========================================================================

TEST(MeshCsgSubtree, CallingTwiceProducesByteIdenticalBuffers) {
    const WorldDesc world = gate_square_world();
    const Aabb bounds = bounds_or_fail(world.sdf, /*root=*/2);

    const MeshData first = mesh_or_fail(world.sdf, /*root=*/2, bounds);
    const MeshData second = mesh_or_fail(world.sdf, /*root=*/2, bounds);

    ASSERT_EQ(first.positions.size(), second.positions.size());
    ASSERT_EQ(first.indices.size(), second.indices.size());

    const std::span<const std::byte> pos_a = as_bytes_of(first.positions);
    const std::span<const std::byte> pos_b = as_bytes_of(second.positions);
    EXPECT_EQ(std::memcmp(pos_a.data(), pos_b.data(), pos_a.size()), 0);

    const std::span<const std::byte> idx_a = as_bytes_of(first.indices);
    const std::span<const std::byte> idx_b = as_bytes_of(second.indices);
    EXPECT_EQ(std::memcmp(idx_a.data(), idx_b.data(), idx_a.size()), 0);
}

// ===========================================================================
// 5. The committed golden manifest -- one entry per (subtree, limits
//    version), snapshot-pinned exactly as test_render_raster.cpp's frame
//    goldens are (a surface-nets vertex/index count has no independent
//    closed form the way tessellate.cpp's primitives do, so there is no
//    second formula to re-derive it from).
// ===========================================================================

namespace {

[[nodiscard]] std::filesystem::path manifest_path() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "render" / "csg" / "manifest.json";
}

// Bumped only alongside a deliberate kCsgMeshDefaults change, together with
// the manifest's own `limits_version` field -- csg_mesh.hpp's RS3 discipline,
// mirroring tessellate.hpp's identical rule for kTessellationDefaults.
constexpr const char* kExpectedLimitsVersion = "kCsgMeshDefaults@2";

// A second self-authored fixture, distinct in shape from gate_square_world()
// (smooth_union rather than subtract, and it stacks a genuine transform on
// one operand) -- broadens the golden corpus beyond one topology without
// coupling it to a shared fixture some OTHER task might reasonably change
// for unrelated reasons (tests/golden/worlds/maximal.world.yaml, used by
// Section 1's split_program() test above, is exactly such a shared
// fixture -- fine to load for a structural check, wrong to pin a byte-exact
// hash against).
WorldDesc smooth_blend_world() {
    WorldBuilder b = base_builder();
    b.cylinder(0.8f, 0.6f)
        .sphere(0.7f, SdfPose{.position = {0.0f, 0.9f, 0.0f}})
        .smooth_union(0.25f);
    return build_or_fail(b);
}

struct GoldenFixture {
    const char* name;
    WorldDesc world;
    uint32_t root_node;
};

std::vector<GoldenFixture> golden_fixtures() {
    std::vector<GoldenFixture> fixtures;
    fixtures.push_back(GoldenFixture{"gate_square", gate_square_world(), 2});
    fixtures.push_back(GoldenFixture{"smooth_blend_cylinder_sphere", smooth_blend_world(), 2});
    return fixtures;
}

}  // namespace

TEST(CsgMeshGolden, MatchesCommittedManifest) {
    const std::filesystem::path path = manifest_path();
    ASSERT_TRUE(std::filesystem::exists(path)) << path.string();

    YAML::Node root = YAML::LoadFile(path.string());
    ASSERT_TRUE(root["limits_version"]) << "manifest missing limits_version";
    EXPECT_EQ(root["limits_version"].as<std::string>(), kExpectedLimitsVersion)
        << "the manifest's limits_version no longer matches this test's own constant -- if "
           "kCsgMeshDefaults changed deliberately, update BOTH together";

    const YAML::Node limits = root["limits"];
    ASSERT_TRUE(limits) << "manifest missing limits";
    EXPECT_FLOAT_EQ(limits["cell_size"].as<float>(), kCsgMeshDefaults.cell_size);
    EXPECT_EQ(limits["min_cells_per_axis"].as<uint32_t>(), kCsgMeshDefaults.min_cells_per_axis);
    EXPECT_EQ(limits["max_cells_per_axis"].as<uint32_t>(), kCsgMeshDefaults.max_cells_per_axis);
    EXPECT_FLOAT_EQ(limits["aabb_margin"].as<float>(), kCsgMeshDefaults.aabb_margin);

    const YAML::Node subtrees = root["subtrees"];
    ASSERT_TRUE(subtrees) << "manifest missing subtrees";

    for (const GoldenFixture& fx : golden_fixtures()) {
        const YAML::Node entry = subtrees[fx.name];
        ASSERT_TRUE(entry) << "manifest missing entry for " << fx.name;

        const Aabb bounds = bounds_or_fail(fx.world.sdf, fx.root_node);
        const MeshData mesh = mesh_or_fail(fx.world.sdf, fx.root_node, bounds);

        EXPECT_EQ(mesh.positions.size(), entry["vertex_count"].as<uint64_t>()) << fx.name;
        EXPECT_EQ(mesh.indices.size(), entry["index_count"].as<uint64_t>()) << fx.name;

        std::vector<std::byte> combined;
        const std::span<const std::byte> pos_bytes = as_bytes_of(mesh.positions);
        const std::span<const std::byte> idx_bytes = as_bytes_of(mesh.indices);
        combined.reserve(pos_bytes.size() + idx_bytes.size());
        combined.insert(combined.end(), pos_bytes.begin(), pos_bytes.end());
        combined.insert(combined.end(), idx_bytes.begin(), idx_bytes.end());

        EXPECT_EQ(sha256_hex(combined), entry["sha256"].as<std::string>())
            << fx.name << ": CSG mesh no longer matches the committed golden";
    }
}
