#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <span>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/tessellate.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// tessellate_primitive() -- S7a Task R2.
//
// Three burdens of proof:
//
//   * EACH OF THE SEVEN KINDS PRODUCES A VALID MESH matching a CLOSED-FORM
//     vertex/index count derived from kTessellationDefaults -- never an
//     observed count taken on faith. expected_vertex_count()/
//     expected_index_count() below re-derive the same formula
//     tessellate.cpp's own generators use, independently spelled, exactly the
//     way test_determinism.cpp's GoldenCorpus test re-derives digests from a
//     second source rather than trusting the one under test.
//
//   * SAME INPUTS TWICE -> BYTE-IDENTICAL positions/indices (constraint 4):
//     the limits table is a fixed constant (RS3), so nothing here may vary
//     between two calls.
//
//   * THE COMMITTED GOLDEN MANIFEST (tests/golden/render/tessellation/
//     manifest.json) still matches: a sha256 of positions+indices per
//     primitive, plus the limits-table values themselves and a
//     `limits_version` tag -- so a DELIBERATE change to kTessellationDefaults
//     is a visible, reviewable manifest edit, and an ACCIDENTAL one fails
//     here. The manifest is plain JSON (a JSON object is valid YAML flow
//     syntax), loaded with yaml-cpp -- already a direct dependency of this
//     target for engine/testing/scenario_file.hpp -- rather than vendoring a
//     second parser for one small file.
// ---------------------------------------------------------------------------

namespace {

using spade::Code;
using spade::Result;
using spade::SdfPrim;
using spade::render::Aabb;
using spade::render::kTessellationDefaults;
using spade::render::MeshData;
using spade::render::tessellate_primitive;
using spade::render::TessellationLimits;

// ===========================================================================
// A tiny, self-contained SHA-256 (FIPS 180-4). Test-only: the golden manifest
// needs a fingerprint STRONGER than the engine's own FNV-1a fold
// (state/snapshot.hpp) specifically BECAUSE it lives in a file a human reads
// and reviews -- 64 hex characters that visibly move on any bit change is the
// point, not collision-hardness this corpus will ever stress. Verified below
// (Sha256Self.MatchesTheStandardTestVectors) against the two textbook
// vectors before anything else in this file trusts it.
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

[[nodiscard]] constexpr uint32_t rotr32(uint32_t x, uint32_t n) noexcept {
    return (x >> n) | (x << (32u - n));
}

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
// Fixtures: one representative parameter set per primitive, chosen to be
// unambiguously non-degenerate (no zero radius/extent) so a bug that
// mishandles an edge case cannot hide behind a trivially-correct answer.
// ===========================================================================

struct PrimFixture {
    SdfPrim kind;
    const char* name;
    glm::vec4 params;
    bool needs_world_bounds;
};

const Aabb kWorldBounds{.min = glm::vec3(-5.0f, -5.0f, -5.0f), .max = glm::vec3(5.0f, 5.0f, 5.0f)};

const std::vector<PrimFixture>& fixtures() {
    static const std::vector<PrimFixture> table = {
        // plane: unit +Y normal, offset 0 -- needs world_bounds (PA-5).
        {SdfPrim::plane, "plane", glm::vec4(0.0f, 1.0f, 0.0f, 0.0f), true},
        {SdfPrim::sphere, "sphere", glm::vec4(1.5f, 0.0f, 0.0f, 0.0f), false},
        {SdfPrim::box, "box", glm::vec4(1.0f, 0.75f, 1.25f, 0.0f), false},
        {SdfPrim::cylinder, "cylinder", glm::vec4(0.6f, 1.2f, 0.0f, 0.0f), false},
        {SdfPrim::capsule, "capsule", glm::vec4(0.4f, 0.9f, 0.0f, 0.0f), false},
        {SdfPrim::torus, "torus", glm::vec4(1.2f, 0.35f, 0.0f, 0.0f), false},
        // heightfield: amplitude/freq_x/freq_z/base_y -- also needs world_bounds
        // (its (x,z) domain is just as analytically unbounded as the plane's).
        {SdfPrim::heightfield, "heightfield", glm::vec4(0.6f, 0.8f, 0.5f, 0.1f), true},
    };
    return table;
}

[[nodiscard]] MeshData mesh_or_fail(const PrimFixture& fx, const TessellationLimits& limits = kTessellationDefaults) {
    const Result<MeshData> mesh = tessellate_primitive(fx.kind, fx.params, kWorldBounds, limits);
    if (!mesh) {
        ADD_FAILURE() << fx.name << ": tessellate_primitive failed: " << mesh.error().context;
        return MeshData{};
    }
    return *mesh;
}

// The closed-form vertex/index counts, re-derived independently from
// kTessellationDefaults's fields -- see tessellate.cpp for the generator each
// formula matches.
[[nodiscard]] uint64_t expected_vertex_count(SdfPrim kind, const TessellationLimits& l) {
    switch (kind) {
        case SdfPrim::plane:
            return static_cast<uint64_t>(l.plane_grid_cells + 1) * (l.plane_grid_cells + 1);
        case SdfPrim::sphere:
            return static_cast<uint64_t>(l.sphere_rings + 1) * (l.sphere_segments + 1);
        case SdfPrim::box:
            return 24;
        case SdfPrim::cylinder:
            return static_cast<uint64_t>(4) * l.circle_segments + 2;
        case SdfPrim::capsule: {
            const uint32_t hemisphere_rings = l.circle_segments / 4;
            return static_cast<uint64_t>(l.circle_segments) * (2 * hemisphere_rings + 4);
        }
        case SdfPrim::torus:
            return static_cast<uint64_t>(l.circle_segments) * l.torus_ring_segments;
        case SdfPrim::heightfield:
            return static_cast<uint64_t>(l.heightfield_cells + 1) * (l.heightfield_cells + 1);
    }
    return 0;
}

[[nodiscard]] uint64_t expected_index_count(SdfPrim kind, const TessellationLimits& l) {
    switch (kind) {
        case SdfPrim::plane:
            return static_cast<uint64_t>(l.plane_grid_cells) * l.plane_grid_cells * 6;
        case SdfPrim::sphere:
            return static_cast<uint64_t>(l.sphere_rings) * l.sphere_segments * 6;
        case SdfPrim::box:
            return 36;
        case SdfPrim::cylinder:
            return static_cast<uint64_t>(12) * l.circle_segments;
        case SdfPrim::capsule: {
            const uint32_t hemisphere_rings = l.circle_segments / 4;
            return static_cast<uint64_t>(6) * l.circle_segments * (1 + 2 * hemisphere_rings);
        }
        case SdfPrim::torus:
            return static_cast<uint64_t>(l.circle_segments) * l.torus_ring_segments * 6;
        case SdfPrim::heightfield:
            return static_cast<uint64_t>(l.heightfield_cells) * l.heightfield_cells * 6;
    }
    return 0;
}

[[nodiscard]] std::filesystem::path manifest_path() {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / "render" / "tessellation" / "manifest.json";
}

// Bumped only alongside a deliberate kTessellationDefaults change, together
// with tests/golden/render/tessellation/manifest.json's own `limits_version`
// field -- see tessellate.hpp's file comment (RS3) for why the table itself
// never changes silently.
constexpr const char* kExpectedLimitsVersion = "kTessellationDefaults@1";

}  // namespace

// ===========================================================================
// 0. The golden manifest's own hash function is trustworthy.
// ===========================================================================

TEST(Sha256Self, MatchesTheStandardTestVectors) {
    const std::string empty;
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(empty.data(), empty.size()))),
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    const std::string abc = "abc";
    EXPECT_EQ(sha256_hex(std::as_bytes(std::span<const char>(abc.data(), abc.size()))),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

// ===========================================================================
// 1. Every one of the seven kinds produces a valid mesh at the closed-form
//    count.
// ===========================================================================

TEST(Tessellate, EachPrimitiveKindProducesAValidMeshMatchingTheClosedForm) {
    for (const PrimFixture& fx : fixtures()) {
        const MeshData mesh = mesh_or_fail(fx);

        EXPECT_FALSE(mesh.positions.empty()) << fx.name;
        EXPECT_FALSE(mesh.indices.empty()) << fx.name;
        ASSERT_EQ(mesh.positions.size(), mesh.normals.size()) << fx.name;

        EXPECT_EQ(mesh.positions.size(), expected_vertex_count(fx.kind, kTessellationDefaults)) << fx.name;
        EXPECT_EQ(mesh.indices.size(), expected_index_count(fx.kind, kTessellationDefaults)) << fx.name;

        ASSERT_EQ(mesh.indices.size() % 3u, 0u) << fx.name << ": index count is not a whole number of triangles";

        for (const uint32_t index : mesh.indices) {
            ASSERT_LT(index, mesh.positions.size()) << fx.name << ": index out of range";
        }

        for (size_t i = 0; i < mesh.positions.size(); ++i) {
            const glm::vec3& p = mesh.positions[i];
            EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
                << fx.name << ": non-finite position at vertex " << i;

            const glm::vec3& n = mesh.normals[i];
            EXPECT_TRUE(std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z))
                << fx.name << ": non-finite normal at vertex " << i;
            EXPECT_NEAR(glm::length(n), 1.0f, 1e-4f) << fx.name << ": normal not unit-length at vertex " << i;
        }
    }
}

TEST(Tessellate, CapsuleCapsAreHemisphericalNotFlat) {
    // The reversed-shortcut requirement (tessellate.hpp's file comment): a
    // capsule's END vertices must reach beyond the cylindrical body's
    // radius*sqrt(2) diagonal in the axial direction the way a flat cylinder
    // cap never would -- concretely, at least one vertex must sit strictly
    // beyond the half_height plane along +/-Y, at a distance from the capsule
    // axis strictly less than the radius (a point ON a hemisphere, not on a
    // flat disc coplanar with half_height).
    const PrimFixture fx{SdfPrim::capsule, "capsule", glm::vec4(0.4f, 0.9f, 0.0f, 0.0f), false};
    const MeshData mesh = mesh_or_fail(fx);

    const float radius = fx.params.x;
    const float half_height = fx.params.y;
    bool found_dome_vertex = false;
    for (const glm::vec3& p : mesh.positions) {
        const float axial_beyond = std::fabs(p.y) - half_height;
        const float radial = std::sqrt(p.x * p.x + p.z * p.z);
        if (axial_beyond > 1e-4f && radial < radius - 1e-4f) {
            found_dome_vertex = true;
            break;
        }
    }
    EXPECT_TRUE(found_dome_vertex) << "no vertex sits on a dome beyond the cylindrical body -- "
                                       "the caps look flat, which is the cylinder shortcut this task reverses";
}

// The stated winding convention (tessellate.cpp's tessellate_box() comment:
// "CCW as seen from outside the box") is a claim about every generator, not
// just box -- and nothing above this test can tell a correctly-wound triangle
// from a backward one: vertex/index counts, index range and unit-normal
// length are all identical either way. For every non-degenerate triangle,
// cross(p1-p0, p2-p0) (the triangle's own face normal, by the same
// right-hand-rule convention `add_triangle`'s argument order encodes) must
// point the same way as the vertices' own stored normals -- summed rather
// than averaged, since only the SIGN of the dot product matters and a flat
// face's three identical normals sum to three times themselves with no
// change of sign.
//
// Degenerate (zero-area) triangles are skipped rather than asserted on: the
// sphere/capsule pole rows deliberately collapse one of each pair's two
// triangles to a single point (this file's own tessellate.cpp comments say
// so), and a cross product near the zero vector has a sign that is pure
// floating-point noise, not a fact about winding.
TEST(Tessellate, TriangleWindingIsConsistentlyOutward) {
    constexpr float kDegenerateAreaEpsilon = 1e-6f;
    for (const PrimFixture& fx : fixtures()) {
        const MeshData mesh = mesh_or_fail(fx);
        ASSERT_EQ(mesh.indices.size() % 3u, 0u) << fx.name;

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
                continue;  // pole-collapsed triangle -- sign is meaningless
            }
            ++non_degenerate_triangles;

            const glm::vec3 stored_normal_sum = mesh.normals[ia] + mesh.normals[ib] + mesh.normals[ic];
            EXPECT_GT(glm::dot(face_normal, stored_normal_sum), 0.0f)
                << fx.name << ": triangle " << (t / 3)
                << " is wound backward -- its face normal points opposite its own vertices' stored normals";
        }
        EXPECT_GT(non_degenerate_triangles, 0u)
            << fx.name << ": every triangle was degenerate -- this test checked nothing for this primitive";
    }
}

// ===========================================================================
// 2. Determinism: identical inputs, byte-identical output (constraint 4).
// ===========================================================================

TEST(Tessellate, CallingTwiceProducesByteIdenticalBuffers) {
    for (const PrimFixture& fx : fixtures()) {
        const MeshData first = mesh_or_fail(fx);
        const MeshData second = mesh_or_fail(fx);

        ASSERT_EQ(first.positions.size(), second.positions.size()) << fx.name;
        ASSERT_EQ(first.indices.size(), second.indices.size()) << fx.name;

        const std::span<const std::byte> pos_a = as_bytes_of(first.positions);
        const std::span<const std::byte> pos_b = as_bytes_of(second.positions);
        EXPECT_EQ(std::memcmp(pos_a.data(), pos_b.data(), pos_a.size()), 0)
            << fx.name << ": positions differ between two calls with identical inputs";

        const std::span<const std::byte> idx_a = as_bytes_of(first.indices);
        const std::span<const std::byte> idx_b = as_bytes_of(second.indices);
        EXPECT_EQ(std::memcmp(idx_a.data(), idx_b.data(), idx_a.size()), 0)
            << fx.name << ": indices differ between two calls with identical inputs";
    }
}

// ===========================================================================
// 3. The committed golden manifest.
// ===========================================================================

TEST(TessellateGolden, MatchesCommittedManifest) {
    const std::filesystem::path path = manifest_path();
    ASSERT_TRUE(std::filesystem::exists(path)) << path.string();

    YAML::Node root = YAML::LoadFile(path.string());
    ASSERT_TRUE(root["limits_version"]) << "manifest missing limits_version";
    EXPECT_EQ(root["limits_version"].as<std::string>(), kExpectedLimitsVersion)
        << "the manifest's limits_version no longer matches this test's own constant -- if "
           "kTessellationDefaults changed deliberately, update BOTH together";

    const YAML::Node limits = root["limits"];
    ASSERT_TRUE(limits) << "manifest missing limits";
    EXPECT_EQ(limits["circle_segments"].as<uint32_t>(), kTessellationDefaults.circle_segments);
    EXPECT_EQ(limits["sphere_rings"].as<uint32_t>(), kTessellationDefaults.sphere_rings);
    EXPECT_EQ(limits["sphere_segments"].as<uint32_t>(), kTessellationDefaults.sphere_segments);
    EXPECT_EQ(limits["torus_ring_segments"].as<uint32_t>(), kTessellationDefaults.torus_ring_segments);
    EXPECT_EQ(limits["heightfield_cells"].as<uint32_t>(), kTessellationDefaults.heightfield_cells);
    EXPECT_EQ(limits["plane_grid_cells"].as<uint32_t>(), kTessellationDefaults.plane_grid_cells);

    const YAML::Node primitives = root["primitives"];
    ASSERT_TRUE(primitives) << "manifest missing primitives";

    for (const PrimFixture& fx : fixtures()) {
        const YAML::Node entry = primitives[fx.name];
        ASSERT_TRUE(entry) << "manifest missing entry for " << fx.name;

        const MeshData mesh = mesh_or_fail(fx);

        EXPECT_EQ(mesh.positions.size(), entry["vertex_count"].as<uint64_t>()) << fx.name;
        EXPECT_EQ(mesh.indices.size(), entry["index_count"].as<uint64_t>()) << fx.name;

        std::vector<std::byte> combined;
        const std::span<const std::byte> pos_bytes = as_bytes_of(mesh.positions);
        const std::span<const std::byte> idx_bytes = as_bytes_of(mesh.indices);
        combined.reserve(pos_bytes.size() + idx_bytes.size());
        combined.insert(combined.end(), pos_bytes.begin(), pos_bytes.end());
        combined.insert(combined.end(), idx_bytes.begin(), idx_bytes.end());

        EXPECT_EQ(sha256_hex(combined), entry["sha256"].as<std::string>())
            << fx.name << ": tessellated mesh no longer matches the committed golden";
    }
}
