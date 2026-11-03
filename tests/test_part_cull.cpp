// sample_parts()'s mesh-box cull (the lead's ask, 2026-10-28; Physics
// reviews): a mesh part whose box is farther than the best distance so far is
// skipped, so a face sample queries a few parts' meshes, not all eleven, and
// the answer is the unculled one bit for bit. mesh_box_distance2()
// (world/mesh.hpp) is the bound; vehicles/part_pose.cpp says why it is exact.
#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "library_drones.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"
#include "world/mesh.hpp"
#include "world/mesh_shapes.hpp"

namespace {

namespace ld = spade::test::library_drones;
namespace vh = spade::vehicles;

[[nodiscard]] uint32_t bits(float v) { return std::bit_cast<uint32_t>(v); }

// The squared distance from p to [lo, hi], spelt as mesh.cpp's box_distance2.
[[nodiscard]] float hand_box_distance2(glm::vec3 p, glm::vec3 lo, glm::vec3 hi) {
    const float dx = p.x < lo.x ? lo.x - p.x : (p.x > hi.x ? p.x - hi.x : 0.0f);
    const float dy = p.y < lo.y ? lo.y - p.y : (p.y > hi.y ? p.y - hi.y : 0.0f);
    const float dz = p.z < lo.z ? lo.z - p.z : (p.z > hi.z ? p.z - hi.z : 0.0f);
    return dx * dx + dy * dy + dz * dz;
}

// sample_parts() as it was before the cull (master c39bcadb), the reference:
// every posed part in declaration order, strictly smaller replacing the
// winner, the gradient's chain and the velocity as part_pose.cpp spells them.
// It also counts what the cull meets at x: a mesh part it would skip, and a
// mesh part that wins over a best already inside another part while x lies
// in its box (the samples an unguarded cull would get wrong).
struct Counts {
    std::size_t culled = 0;
    std::size_t overlap_wins = 0;
};

[[nodiscard]] vh::PartSample unculled(const vh::ModelType& model, const vh::PosedParts& posed, glm::vec3 x,
                                      Counts& counts) {
    vh::PartSample out{};
    std::size_t winner = posed.parts.size();
    glm::vec3 part_gradient(0.0f);
    for (std::size_t j = 0; j < posed.parts.size(); ++j) {
        const vh::PosedPart& p = posed.parts[j];
        const spade::SdfTransform& t = p.world_to_part;
        const glm::vec3 local(t.world_to_local * glm::vec4(x, 1.0f));
        const vh::PartDesc& part = model.parts[p.part];
        float b2 = -1.0f;
        if (part.mesh != nullptr) {
            b2 = spade::mesh_box_distance2(*part.mesh, local);
            if (b2 > 0.0f && std::sqrt(b2) * t.scale > out.distance) ++counts.culled;
        }
        const spade::SdfSample s = vh::part_sample(part, local);
        const float distance = s.distance * t.scale;
        if (distance < out.distance) {
            if (b2 == 0.0f && out.distance < 0.0f) ++counts.overlap_wins;
            out.distance = distance;
            part_gradient = s.gradient;
            winner = j;
        }
    }
    if (winner == posed.parts.size()) return out;
    const vh::PosedPart& p = posed.parts[winner];
    out.gradient = (glm::transpose(glm::mat3(p.world_to_part.world_to_local)) * part_gradient) * p.world_to_part.scale;
    out.owner = p.part;
    out.surface = model.parts[p.part].surface.kind;
    out.velocity = vh::part_wall_velocity(posed, winner, x);
    return out;
}

}  // namespace

// The bound: outside a mesh's box it is the box's squared distance, spelt as
// the query's; inside, 0; for a non-finite point, 0; and its square root is
// never more than the field's |distance|, so a part it culls could not have
// come nearer. A rounded box (rounded edges inside its box's corners), swept
// on a lattice that straddles every face, inside and out.
TEST(PartCull, MeshBoxDistanceBoundsTheField) {
    const spade::Result<spade::shapes::GeneratedMesh> box = spade::shapes::rounded_box({0.18, 0.05, 0.14}, 0.008, 4);
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const spade::TriangleMesh& mesh = *box->mesh;
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (const glm::vec3& v : mesh.positions()) {
        lo = glm::min(lo, v);
        hi = glm::max(hi, v);
    }
    std::size_t outside = 0;
    std::size_t inside = 0;
    for (int i = -6; i <= 6; ++i) {
        for (int j = -6; j <= 6; ++j) {
            for (int k = -6; k <= 6; ++k) {
                const glm::vec3 p(0.0175f * static_cast<float>(i), 0.0055f * static_cast<float>(j),
                                  0.0135f * static_cast<float>(k));
                const float b2 = spade::mesh_box_distance2(mesh, p);
                const float want = hand_box_distance2(p, lo, hi);
                EXPECT_EQ(bits(b2), bits(want)) << "at (" << p.x << ", " << p.y << ", " << p.z << ")";
                const float d = spade::mesh_sample(mesh, p).distance;
                EXPECT_LE(std::sqrt(b2), std::fabs(d)) << "at (" << p.x << ", " << p.y << ", " << p.z << ")";
                (want > 0.0f ? outside : inside) += 1;
            }
        }
    }
    EXPECT_GT(outside, 0u);
    EXPECT_GT(inside, 0u);
    EXPECT_EQ(spade::mesh_box_distance2(mesh, glm::vec3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)), 0.0f);
    EXPECT_EQ(spade::mesh_box_distance2(mesh, glm::vec3(std::numeric_limits<float>::infinity(), 0.0f, 0.0f)), 0.0f);
}

namespace {

struct CullCase {
    std::string_view stem;
};

void PrintTo(const CullCase& c, std::ostream* out) { *out << c.stem; }

class PartCullOnALibraryDrone : public testing::TestWithParam<CullCase> {};

}  // namespace

// The cull keeps sample_parts() bit for bit (the lead's ruling (a),
// 2026-10-28): on every 13th MAC face sample of a library drone at 4 mm, posed
// upright at rest with rigid_only, in the solids test's grid (its box and
// 3 dx more on every side; the Phantom's body top on a plane of cell centres),
// the culled answer -- distance, gradient, owner, surface and velocity -- is
// the unculled one's bits. The samples include parts the cull skips and
// samples inside two parts at once where a later mesh part wins, which a cull
// without the guard would skip.
TEST_P(PartCullOnALibraryDrone, KeepsSamplePartsBitForBit) {
    const std::string_view stem = GetParam().stem;
    const spade::Result<ld::Committed> lib = ld::committed_library();
    ASSERT_TRUE(lib.has_value()) << lib.error().context;
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, stem);
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    const auto compiled = vh::compile_airframe(drone->spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const vh::ModelType& model = compiled->model;
    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const auto posed = vh::posed_parts(model, body, {}, {}, vh::PartFilter::rigid_only);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;

    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    const vh::AirflowSolids world(model, *posed, glm::vec3(0.0f));
    const auto parts = world.parts();
    ASSERT_TRUE(parts.has_value()) << parts.error().context;
    for (const spade::physics::airflow::SolidPart& p : *parts) {
        lo = glm::min(lo, p.box_min);
        hi = glm::max(hi, p.box_max);
    }
    constexpr float dx = 0.004f;
    const glm::vec3 origin = lo - glm::vec3(3.0f * dx);
    const glm::vec3 size = hi - lo + glm::vec3(6.0f * dx);
    const int n[3] = {static_cast<int>(std::ceil(size.x / dx)), static_cast<int>(std::ceil(size.y / dx)),
                      static_cast<int>(std::ceil(size.z / dx))};

    constexpr std::size_t kEvery = 13;
    std::size_t seen = 0;
    std::size_t compared = 0;
    std::size_t mismatches = 0;
    Counts counts;
    for (int c = 0; c < 3; ++c) {
        for (int i = 0; i < n[0] + (c == 0 ? 1 : 0); ++i) {
            for (int j = 0; j < n[1] + (c == 1 ? 1 : 0); ++j) {
                for (int k = 0; k < n[2] + (c == 2 ? 1 : 0); ++k) {
                    if (seen++ % kEvery != 0) continue;
                    const glm::vec3 at(static_cast<float>(i) + (c == 0 ? 0.0f : 0.5f),
                                       static_cast<float>(j) + (c == 1 ? 0.0f : 0.5f),
                                       static_cast<float>(k) + (c == 2 ? 0.0f : 0.5f));
                    const glm::vec3 x = origin + at * dx;
                    const vh::PartSample want = unculled(model, *posed, x, counts);
                    const vh::PartSample got = vh::sample_parts(model, *posed, x);
                    ++compared;
                    const bool same = bits(got.distance) == bits(want.distance) &&
                                      bits(got.gradient.x) == bits(want.gradient.x) &&
                                      bits(got.gradient.y) == bits(want.gradient.y) &&
                                      bits(got.gradient.z) == bits(want.gradient.z) && got.owner == want.owner &&
                                      got.surface == want.surface && bits(got.velocity.x) == bits(want.velocity.x) &&
                                      bits(got.velocity.y) == bits(want.velocity.y) &&
                                      bits(got.velocity.z) == bits(want.velocity.z);
                    if (!same && ++mismatches <= 5) {
                        ADD_FAILURE() << "component " << c << " sample (" << i << ", " << j << ", " << k
                                      << "): culled distance " << got.distance << " owner " << got.owner
                                      << ", unculled " << want.distance << " owner " << want.owner;
                    }
                }
            }
        }
    }
    std::printf("%.*s: %zu of %zu face samples compared, %zu mismatches; %zu mesh parts culled, %zu overlap wins\n",
                static_cast<int>(stem.size()), stem.data(), compared, seen, mismatches, counts.culled,
                counts.overlap_wins);
    EXPECT_EQ(mismatches, 0u);
    EXPECT_GT(counts.culled, compared) << "the cull skips more than one part a sample";
    EXPECT_GT(counts.overlap_wins, 0u) << "no sample inside two parts where a later mesh part wins";
}

INSTANTIATE_TEST_SUITE_P(EachDrone, PartCullOnALibraryDrone,
                         testing::Values(CullCase{"phantom3_class"}, CullCase{"basic_quad_competition"}),
                         [](const testing::TestParamInfo<CullCase>& info) { return std::string(info.param.stem); });
