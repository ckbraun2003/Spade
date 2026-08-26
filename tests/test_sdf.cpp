#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// SDF program + WorldBuilder tests.
//
// Distance checks compare against INDEPENDENT reference formulations (a
// four-case cylinder, a clamp-to-box nearest point, a point-segment projection)
// rather than restating the implementation's branch-free closed forms -- a test
// that repeats the code under test only proves it is self-consistent. Where the
// primitive is posed, the reference is written in WORLD space so the transform
// pipeline (pre-inverted matrix + uniform scale) is checked too.
//
// Every sample set is deterministic: a hand-rolled LCG, fixed seeds. No
// std::random_device -- a test that draws differently per run is a flake
// generator, and engine randomness belongs to spade::rng.
// ---------------------------------------------------------------------------

namespace {

using spade::Capacities;
using spade::Code;
using spade::Environment;
using spade::SdfPose;
using spade::SdfProgram;
using spade::SpawnPoint;
using spade::WorldBuilder;
using spade::WorldDesc;

constexpr int kSamples = 100;
constexpr float kDistTol = 1e-5f;
constexpr float kGradTol = 1e-2f;

// Numerical Recipes LCG. Deterministic, seeded per test.
class Lcg {
public:
    explicit Lcg(uint32_t seed) : state_(seed) {}

    float uniform(float lo, float hi) {
        state_ = state_ * 1664525u + 1013904223u;
        const float u = static_cast<float>(state_ >> 8) * (1.0f / 16777216.0f);  // [0,1)
        return lo + (hi - lo) * u;
    }

    // Component order pinned so the sample set cannot silently reshuffle.
    glm::vec3 point(float extent) {
        const float x = uniform(-extent, extent);
        const float y = uniform(-extent, extent);
        const float z = uniform(-extent, extent);
        return glm::vec3(x, y, z);
    }

private:
    uint32_t state_;
};

// A builder with the world-level requirements already satisfied, so SDF tests
// only say what they are testing.
WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("test").capacities(Capacities{.bodies = 1,
                                        .force_elements = 1,
                                        .sensors = 1,
                                        .contacts = 1});
    return b;
}

SdfProgram sdf_of(const WorldBuilder& b) {
    const spade::Result<WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "builder failed: " << world.error().context;
        return SdfProgram{};
    }
    return world->sdf;
}

// --- independent reference distances (all local/world space as noted) -------

float plane_reference(glm::vec3 p, glm::vec3 n, float offset) {
    // Geometric: project onto the plane, measure, then re-sign.
    const float signed_gap = glm::dot(p, n) - offset;
    const glm::vec3 foot = p - signed_gap * n;
    const float len = glm::length(p - foot);
    return signed_gap < 0.0f ? -len : len;
}

float box_reference(glm::vec3 p, glm::vec3 half_extents) {
    const glm::vec3 a = glm::abs(p);
    if (a.x <= half_extents.x && a.y <= half_extents.y && a.z <= half_extents.z) {
        // Interior: negative distance to the nearest face.
        const float dx = half_extents.x - a.x;
        const float dy = half_extents.y - a.y;
        const float dz = half_extents.z - a.z;
        return -std::min(dx, std::min(dy, dz));
    }
    // Exterior: Euclidean distance to the closest point of the box.
    const glm::vec3 nearest = glm::clamp(p, -half_extents, half_extents);
    return glm::length(p - nearest);
}

float cylinder_reference(glm::vec3 p, float radius, float half_height) {
    const float radial = std::sqrt(p.x * p.x + p.z * p.z) - radius;
    const float axial = std::abs(p.y) - half_height;
    if (radial <= 0.0f && axial <= 0.0f) {
        return std::max(radial, axial);  // interior
    }
    if (radial <= 0.0f) {
        return axial;  // beyond a cap, within the radius
    }
    if (axial <= 0.0f) {
        return radial;  // beside the wall, between the caps
    }
    return std::sqrt(radial * radial + axial * axial);  // past the rim
}

float capsule_reference(glm::vec3 p, float radius, float half_height) {
    const glm::vec3 a(0.0f, -half_height, 0.0f);
    const glm::vec3 b(0.0f, half_height, 0.0f);
    const glm::vec3 ab = b - a;
    const float t = glm::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f);
    return glm::length(p - (a + t * ab)) - radius;
}

float torus_reference(glm::vec3 p, float major, float minor) {
    // Distance to the ring's centre circle (radius `major`, in XZ), minus the
    // tube radius. Built from the explicit nearest point on that circle.
    const glm::vec2 xz(p.x, p.z);
    const float len = glm::length(xz);
    glm::vec3 nearest(major, 0.0f, 0.0f);  // degenerate on the axis: any ring point
    if (len > 0.0f) {
        const glm::vec2 scaled = xz * (major / len);
        nearest = glm::vec3(scaled.x, 0.0f, scaled.y);
    }
    return glm::length(p - nearest) - minor;
}

// Central differences over a whole program, for the stencil pin and for
// cross-checking the analytic gradients.
glm::vec3 program_central_difference(const SdfProgram& prog, glm::vec3 p, float h) {
    const float inv_2h = 0.5f / h;
    const float dx = spade::eval(prog, glm::vec3(p.x + h, p.y, p.z)) -
                     spade::eval(prog, glm::vec3(p.x - h, p.y, p.z));
    const float dy = spade::eval(prog, glm::vec3(p.x, p.y + h, p.z)) -
                     spade::eval(prog, glm::vec3(p.x, p.y - h, p.z));
    const float dz = spade::eval(prog, glm::vec3(p.x, p.y, p.z + h)) -
                     spade::eval(prog, glm::vec3(p.x, p.y, p.z - h));
    return glm::vec3(dx * inv_2h, dy * inv_2h, dz * inv_2h);
}

}  // namespace

// ===========================================================================
// Primitive distances vs closed form -- 100 deterministic samples each.
// ===========================================================================

TEST(SdfPrimitive, PlaneMatchesClosedForm) {
    const glm::vec3 n = glm::normalize(glm::vec3(1.0f, 2.0f, -3.0f));
    const float offset = 0.5f;
    WorldBuilder b = base_builder();
    b.plane(n * 4.0f, offset);  // deliberately un-normalized: the builder normalizes
    const SdfProgram prog = sdf_of(b);

    Lcg rng(0x5eed0001u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(3.0f);
        EXPECT_NEAR(spade::eval(prog, p), plane_reference(p, n, offset), kDistTol) << "i=" << i;
    }
}

TEST(SdfPrimitive, PosedPlaneMatchesWorldSpaceClosedForm) {
    // A posed plane is another plane: normal R*n, offset s*o + dot(position, R*n).
    const glm::vec3 n_local = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
    const float o_local = 0.25f;
    const SdfPose pose{.position = glm::vec3(1.0f, -2.0f, 0.5f),
                       .rotation = glm::angleAxis(glm::radians(35.0f),
                                                  glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f))),
                       .scale = 2.0f};
    WorldBuilder b = base_builder();
    b.plane(n_local, o_local, pose);
    const SdfProgram prog = sdf_of(b);

    const glm::vec3 n_world = pose.rotation * n_local;
    const float o_world = pose.scale * o_local + glm::dot(pose.position, n_world);

    Lcg rng(0x5eed0002u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(4.0f);
        EXPECT_NEAR(spade::eval(prog, p), plane_reference(p, n_world, o_world), kDistTol)
            << "i=" << i;
    }
}

TEST(SdfPrimitive, SphereMatchesClosedFormUnderPoseAndScale) {
    const float radius = 0.75f;
    const SdfPose pose{.position = glm::vec3(-1.0f, 0.5f, 2.0f),
                       .rotation = glm::angleAxis(glm::radians(-70.0f),
                                                  glm::normalize(glm::vec3(0.0f, 1.0f, 1.0f))),
                       .scale = 1.5f};
    WorldBuilder b = base_builder();
    b.sphere(radius, pose);
    const SdfProgram prog = sdf_of(b);

    Lcg rng(0x5eed0003u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(4.0f);
        // Rotation cannot matter for a sphere; uniform scale multiplies both the
        // radius and the distance.
        const float expected = glm::length(p - pose.position) - radius * pose.scale;
        EXPECT_NEAR(spade::eval(prog, p), expected, kDistTol) << "i=" << i;
    }
}

TEST(SdfPrimitive, BoxMatchesClosedFormInsideAndOutside) {
    const glm::vec3 half(0.8f, 1.2f, 0.4f);
    const SdfPose pose{.position = glm::vec3(0.3f, -0.6f, 0.2f),
                       .rotation = glm::angleAxis(glm::radians(50.0f),
                                                  glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))),
                       .scale = 1.25f};
    WorldBuilder b = base_builder();
    b.box(half, pose);
    const SdfProgram prog = sdf_of(b);

    const glm::mat3 rot = glm::mat3_cast(pose.rotation);
    int interior = 0;
    Lcg rng(0x5eed0004u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(2.5f);
        const glm::vec3 local = glm::transpose(rot) * (p - pose.position) / pose.scale;
        const float expected = pose.scale * box_reference(local, half);
        EXPECT_NEAR(spade::eval(prog, p), expected, kDistTol) << "i=" << i;
        if (expected < 0.0f) {
            ++interior;
        }
    }
    EXPECT_GT(interior, 0) << "sample set never landed inside the box -- test is vacuous";
}

TEST(SdfPrimitive, CylinderMatchesClosedForm) {
    const float radius = 0.9f;
    const float half_height = 1.1f;
    WorldBuilder b = base_builder();
    b.cylinder(radius, half_height);
    const SdfProgram prog = sdf_of(b);

    int interior = 0;
    Lcg rng(0x5eed0005u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(2.0f);
        const float expected = cylinder_reference(p, radius, half_height);
        EXPECT_NEAR(spade::eval(prog, p), expected, kDistTol) << "i=" << i;
        if (expected < 0.0f) {
            ++interior;
        }
    }
    EXPECT_GT(interior, 0);
}

TEST(SdfPrimitive, CapsuleMatchesClosedForm) {
    const float radius = 0.35f;
    const float half_height = 0.8f;
    WorldBuilder b = base_builder();
    b.capsule(radius, half_height);
    const SdfProgram prog = sdf_of(b);

    int interior = 0;
    Lcg rng(0x5eed0006u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(1.6f);
        const float expected = capsule_reference(p, radius, half_height);
        EXPECT_NEAR(spade::eval(prog, p), expected, kDistTol) << "i=" << i;
        if (expected < 0.0f) {
            ++interior;
        }
    }
    EXPECT_GT(interior, 0);
}

TEST(SdfPrimitive, TorusMatchesClosedForm) {
    const float major = 1.5f;
    const float minor = 0.3f;
    WorldBuilder b = base_builder();
    b.torus(major, minor);
    const SdfProgram prog = sdf_of(b);

    int interior = 0;
    Lcg rng(0x5eed0007u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(2.5f);
        const float expected = torus_reference(p, major, minor);
        EXPECT_NEAR(spade::eval(prog, p), expected, kDistTol) << "i=" << i;
        if (expected < 0.0f) {
            ++interior;
        }
    }
    EXPECT_GT(interior, 0);
}

// The heightfield is the one primitive whose distance is a conservative BOUND,
// not an exact metric (sdf.hpp documents why), so it is checked against its
// defining formula plus the three properties that make the bound usable:
// correct sign, never over-estimating the vertical gap, and 1-Lipschitz.
TEST(SdfPrimitive, HeightfieldSignBoundAndFormula) {
    const float amp = 0.5f;
    const glm::vec2 freq(1.0f, 0.7f);
    const float base_y = -1.0f;
    WorldBuilder b = base_builder();
    b.heightfield(amp, freq, base_y);
    const SdfProgram prog = sdf_of(b);

    const float slope = std::abs(amp) * std::max(std::abs(freq.x), std::abs(freq.y));
    const float lipschitz = std::sqrt(1.0f + slope * slope);

    int below = 0;
    int above = 0;
    Lcg rng(0x5eed0008u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(3.0f);
        const float surface = base_y + amp * std::sin(freq.x * p.x) * std::sin(freq.y * p.z);
        const float gap = p.y - surface;
        const float d = spade::eval(prog, p);

        EXPECT_NEAR(d, gap / lipschitz, kDistTol) << "i=" << i;
        EXPECT_EQ(d < 0.0f, gap < 0.0f) << "sign disagrees with the surface, i=" << i;
        EXPECT_LE(std::abs(d), std::abs(gap) + kDistTol) << "bound over-estimates, i=" << i;
        EXPECT_LE(glm::length(spade::gradient(prog, p)), 1.0f + kGradTol) << "i=" << i;

        if (gap < 0.0f) {
            ++below;
        } else {
            ++above;
        }
    }
    EXPECT_GT(below, 0);
    EXPECT_GT(above, 0);
}

TEST(SdfPrimitive, FlatHeightfieldIsExactlyAPlane) {
    // amplitude 0 collapses the Lipschitz divisor to 1, so the field must become
    // the exact plane y = base.
    const float base_y = 0.25f;
    WorldBuilder b = base_builder();
    b.heightfield(0.0f, glm::vec2(2.0f, 3.0f), base_y);
    const SdfProgram prog = sdf_of(b);

    Lcg rng(0x5eed0009u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(3.0f);
        EXPECT_NEAR(spade::eval(prog, p), p.y - base_y, kDistTol) << "i=" << i;
        EXPECT_NEAR(glm::length(spade::gradient(prog, p)), 1.0f, kGradTol) << "i=" << i;
    }
}

// ===========================================================================
// Transforms
// ===========================================================================

TEST(SdfTransformNode, RotationReorientsLocalAxes) {
    // A long-X box rotated 90 degrees about Z becomes a long-Y box.
    WorldBuilder b = base_builder();
    b.box(glm::vec3(2.0f, 0.5f, 0.5f),
          SdfPose{.position = glm::vec3(0.0f),
                  .rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
                  .scale = 1.0f});
    const SdfProgram prog = sdf_of(b);

    EXPECT_NEAR(spade::eval(prog, glm::vec3(0.0f, 1.5f, 0.0f)), -0.5f, kDistTol);  // inside
    EXPECT_NEAR(spade::eval(prog, glm::vec3(1.5f, 0.0f, 0.0f)), 1.0f, kDistTol);   // outside
}

TEST(SdfTransformNode, UniformScaleScalesDistanceAndPreservesGradientLength) {
    const SdfPose pose{.position = glm::vec3(1.0f, 2.0f, 3.0f),
                       .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                       .scale = 2.0f};
    WorldBuilder b = base_builder();
    b.sphere(1.0f, pose);
    const SdfProgram prog = sdf_of(b);

    EXPECT_NEAR(spade::eval(prog, glm::vec3(1.0f, 2.0f, 3.0f)), -2.0f, kDistTol);  // centre
    EXPECT_NEAR(spade::eval(prog, glm::vec3(4.0f, 2.0f, 3.0f)), 1.0f, kDistTol);   // 3 out, r=2

    const glm::vec3 g = spade::gradient(prog, glm::vec3(4.0f, 2.0f, 3.0f));
    EXPECT_NEAR(glm::length(g), 1.0f, kGradTol);
    EXPECT_NEAR(g.x, 1.0f, kGradTol);
}

// ===========================================================================
// CSG identities
// ===========================================================================

namespace {

// Two overlapping primitives, plus the single-node programs for each, so the
// combined field can be checked against the operands' own fields.
struct CsgFixture {
    SdfProgram a;
    SdfProgram b;
};

CsgFixture make_operands() {
    WorldBuilder ba = base_builder();
    ba.sphere(1.0f, SdfPose{.position = glm::vec3(-0.4f, 0.0f, 0.0f),
                            .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            .scale = 1.0f});
    WorldBuilder bb = base_builder();
    bb.box(glm::vec3(0.7f, 0.5f, 0.9f),
           SdfPose{.position = glm::vec3(0.5f, 0.1f, 0.0f),
                   .rotation = glm::angleAxis(glm::radians(20.0f), glm::vec3(0.0f, 1.0f, 0.0f)),
                   .scale = 1.0f});
    return CsgFixture{sdf_of(ba), sdf_of(bb)};
}

// The same two operands, combined by `op`, as one program.
SdfProgram make_combined(spade::SdfOp op, float k = 0.0f) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f, SdfPose{.position = glm::vec3(-0.4f, 0.0f, 0.0f),
                           .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                           .scale = 1.0f});
    b.box(glm::vec3(0.7f, 0.5f, 0.9f),
          SdfPose{.position = glm::vec3(0.5f, 0.1f, 0.0f),
                  .rotation = glm::angleAxis(glm::radians(20.0f), glm::vec3(0.0f, 1.0f, 0.0f)),
                  .scale = 1.0f});
    switch (op) {
        case spade::SdfOp::union_:
            b.union_();
            break;
        case spade::SdfOp::intersect:
            b.intersect();
            break;
        case spade::SdfOp::subtract:
            b.subtract();
            break;
        case spade::SdfOp::smooth_union:
            b.smooth_union(k);
            break;
        case spade::SdfOp::none:
            break;
    }
    return sdf_of(b);
}

}  // namespace

TEST(SdfCsg, UnionIsMinIntersectIsMaxSubtractIsMaxNegated) {
    const CsgFixture ops = make_operands();
    const SdfProgram u = make_combined(spade::SdfOp::union_);
    const SdfProgram i = make_combined(spade::SdfOp::intersect);
    const SdfProgram s = make_combined(spade::SdfOp::subtract);

    Lcg rng(0x5eed0010u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        EXPECT_EQ(spade::eval(u, p), std::min(da, db)) << "n=" << n;
        EXPECT_EQ(spade::eval(i, p), std::max(da, db)) << "n=" << n;
        // Postfix "a b subtract" removes the operand added SECOND.
        EXPECT_EQ(spade::eval(s, p), std::max(da, -db)) << "n=" << n;
    }
}

TEST(SdfCsg, SmoothUnionIsBoundedByMinAndDegeneratesToIt) {
    const CsgFixture ops = make_operands();
    const float k = 0.3f;
    const SdfProgram smooth = make_combined(spade::SdfOp::smooth_union, k);
    const SdfProgram tiny = make_combined(spade::SdfOp::smooth_union, 1e-6f);
    const SdfProgram zero = make_combined(spade::SdfOp::smooth_union, 0.0f);
    const SdfProgram hard = make_combined(spade::SdfOp::union_);

    Lcg rng(0x5eed0011u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float m = std::min(spade::eval(ops.a, p), spade::eval(ops.b, p));
        const float d = spade::eval(smooth, p);
        // The quadratic smooth-min never exceeds min and never dips more than
        // k/4 below it: d = min - k*(1-h)^2 with h in [1/2, 1].
        EXPECT_LE(d, m + kDistTol) << "n=" << n;
        EXPECT_GE(d, m - k * 0.25f - kDistTol) << "n=" << n;
        // Vanishing blend radius -> plain union; k == 0 is exactly plain union.
        EXPECT_NEAR(spade::eval(tiny, p), m, 1e-6f) << "n=" << n;
        EXPECT_EQ(spade::eval(zero, p), spade::eval(hard, p)) << "n=" << n;
    }
}

TEST(SdfCsg, SmoothUnionIsStrictlyBelowMinWhereOperandsAreClose) {
    // Guards against a "smooth" union that silently degenerated into min.
    const CsgFixture ops = make_operands();
    const float k = 0.5f;
    const SdfProgram smooth = make_combined(spade::SdfOp::smooth_union, k);

    int blended = 0;
    Lcg rng(0x5eed0012u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da - db) < 0.5f * k) {
            EXPECT_LT(spade::eval(smooth, p), std::min(da, db)) << "n=" << n;
            ++blended;
        }
    }
    EXPECT_GT(blended, 0) << "no sample landed in the blend band -- test is vacuous";
}

// ===========================================================================
// Gradients
// ===========================================================================

TEST(SdfGradient, MagnitudeIsOneOffSurfaceForExactPrimitives) {
    struct Case {
        const char* label;
        SdfProgram prog;
        float extent;
        float axis_guard;  // skip points within this of the local Y axis (torus medial set)
    };

    WorldBuilder bp = base_builder();
    bp.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    WorldBuilder bs = base_builder();
    bs.sphere(0.8f);
    WorldBuilder bb = base_builder();
    bb.box(glm::vec3(0.6f, 0.4f, 0.9f));
    WorldBuilder bc = base_builder();
    bc.cylinder(0.7f, 1.0f);
    WorldBuilder bk = base_builder();
    bk.capsule(0.4f, 0.7f);
    WorldBuilder bt = base_builder();
    bt.torus(1.2f, 0.25f);

    const Case cases[] = {
        {"plane", sdf_of(bp), 3.0f, 0.0f},   {"sphere", sdf_of(bs), 3.0f, 0.0f},
        {"box", sdf_of(bb), 3.0f, 0.0f},     {"cylinder", sdf_of(bc), 3.0f, 0.0f},
        {"capsule", sdf_of(bk), 3.0f, 0.0f}, {"torus", sdf_of(bt), 3.0f, 0.2f},
    };

    for (const Case& c : cases) {
        int checked = 0;
        Lcg rng(0x5eed0020u);
        for (int i = 0; i < kSamples; ++i) {
            const glm::vec3 p = rng.point(c.extent);
            // Exterior, comfortably off the surface: the distance field of a
            // convex primitive is smooth and exactly metric there. Interior
            // fields have creases (the medial axis), where the gradient is
            // legitimately discontinuous and shorter than 1 under a stencil.
            if (spade::eval(c.prog, p) < 0.05f) {
                continue;
            }
            if (c.axis_guard > 0.0f && glm::length(glm::vec2(p.x, p.z)) < c.axis_guard) {
                continue;
            }
            EXPECT_NEAR(glm::length(spade::gradient(c.prog, p)), 1.0f, kGradTol)
                << c.label << " i=" << i;
            ++checked;
        }
        EXPECT_GT(checked, 10) << c.label << ": too few usable samples";
    }
}

TEST(SdfGradient, PosedNonAnalyticPrimitiveKeepsUnitMagnitude) {
    // The central-difference kinds compute their gradient in LOCAL space, so a
    // rotated + scaled node exercises the chain rule (transpose(mat3(inverse))
    // times the uniform scale). That product must come out as a pure rotation:
    // if the scale factor were dropped or applied twice, magnitude would come
    // back as 1/s or s instead of 1.
    const SdfPose pose{.position = glm::vec3(0.4f, -0.9f, 1.1f),
                       .rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
                       .scale = 2.0f};
    WorldBuilder b = base_builder();
    b.torus(1.2f, 0.25f, pose);
    const SdfProgram prog = sdf_of(b);

    const glm::mat3 rot = glm::mat3_cast(pose.rotation);
    int checked = 0;
    Lcg rng(0x5eed0026u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(6.0f);
        if (spade::eval(prog, p) < 0.05f) {
            continue;
        }
        // The torus's exterior medial set is its own axis -- guard in the node's
        // local frame, where that axis is +Y.
        const glm::vec3 local = glm::transpose(rot) * (p - pose.position) / pose.scale;
        if (glm::length(glm::vec2(local.x, local.z)) < 0.2f) {
            continue;
        }
        EXPECT_NEAR(glm::length(spade::gradient(prog, p)), 1.0f, kGradTol) << "i=" << i;
        ++checked;
    }
    EXPECT_GT(checked, 10);
}

TEST(SdfGradient, AnalyticKindsAgreeWithCentralDifferences) {
    // plane/sphere/box take the analytic path; it must land on the same value a
    // central difference of the whole program does.
    WorldBuilder bp = base_builder();
    bp.plane(glm::normalize(glm::vec3(1.0f, 3.0f, -2.0f)), 0.4f);
    WorldBuilder bs = base_builder();
    bs.sphere(0.8f, SdfPose{.position = glm::vec3(0.2f, -0.3f, 0.1f),
                            .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            .scale = 1.3f});
    WorldBuilder bb = base_builder();
    bb.box(glm::vec3(0.6f, 0.4f, 0.9f),
           SdfPose{.position = glm::vec3(-0.2f, 0.4f, 0.0f),
                   .rotation = glm::angleAxis(glm::radians(25.0f), glm::vec3(0.0f, 1.0f, 0.0f)),
                   .scale = 1.0f});
    const SdfProgram progs[] = {sdf_of(bp), sdf_of(bs), sdf_of(bb)};

    for (const SdfProgram& prog : progs) {
        Lcg rng(0x5eed0021u);
        for (int i = 0; i < kSamples; ++i) {
            const glm::vec3 p = rng.point(3.0f);
            if (spade::eval(prog, p) < 0.1f) {
                continue;  // creases and the surface itself are not differentiable
            }
            const glm::vec3 analytic = spade::gradient(prog, p);
            const glm::vec3 numeric = program_central_difference(prog, p, 1e-3f);
            EXPECT_NEAR(analytic.x, numeric.x, 1e-3f) << "i=" << i;
            EXPECT_NEAR(analytic.y, numeric.y, 1e-3f) << "i=" << i;
            EXPECT_NEAR(analytic.z, numeric.z, 1e-3f) << "i=" << i;
        }
    }
}

TEST(SdfGradient, SphereGradientPointsAwayFromCentre) {
    const glm::vec3 centre(0.5f, -1.0f, 0.25f);
    WorldBuilder b = base_builder();
    b.sphere(0.6f, SdfPose{.position = centre,
                           .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                           .scale = 1.0f});
    const SdfProgram prog = sdf_of(b);

    Lcg rng(0x5eed0022u);
    for (int i = 0; i < kSamples; ++i) {
        const glm::vec3 p = rng.point(3.0f);
        if (glm::length(p - centre) < 1e-3f) {
            continue;
        }
        const glm::vec3 expected = glm::normalize(p - centre);
        const glm::vec3 g = spade::gradient(prog, p);
        EXPECT_NEAR(g.x, expected.x, kGradTol) << "i=" << i;
        EXPECT_NEAR(g.y, expected.y, kGradTol) << "i=" << i;
        EXPECT_NEAR(g.z, expected.z, kGradTol) << "i=" << i;
    }
}

TEST(SdfGradient, CentralDifferenceStepIsPinnedAt1e3) {
    // The torus takes the central-difference path. Its gradient must reproduce
    // the h = 1e-3 stencil (positive control) and must NOT reproduce a coarser
    // one (negative control) -- otherwise a silent change of the pinned step
    // would slip through and CPU/GPU contact normals would drift apart.
    WorldBuilder b = base_builder();
    b.torus(1.2f, 0.25f);
    const SdfProgram prog = sdf_of(b);

    const glm::vec3 p(0.9f, 0.35f, 0.15f);
    const glm::vec3 g = spade::gradient(prog, p);
    const glm::vec3 pinned = program_central_difference(prog, p, spade::kSdfGradientStep);
    const glm::vec3 coarse = program_central_difference(prog, p, 1e-1f);

    EXPECT_FLOAT_EQ(g.x, pinned.x);
    EXPECT_FLOAT_EQ(g.y, pinned.y);
    EXPECT_FLOAT_EQ(g.z, pinned.z);
    EXPECT_GT(glm::length(g - coarse), 1e-4f) << "a 100x coarser stencil produced the same answer";
    EXPECT_FLOAT_EQ(spade::kSdfGradientStep, 1e-3f);
}

TEST(SdfGradient, SelectingOpsCarryTheSelectedBranchGradient) {
    const CsgFixture ops = make_operands();
    const SdfProgram u = make_combined(spade::SdfOp::union_);

    Lcg rng(0x5eed0023u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da - db) < 1e-3f) {
            continue;  // on the crease either branch is admissible
        }
        const glm::vec3 expected =
            da < db ? spade::gradient(ops.a, p) : spade::gradient(ops.b, p);
        const glm::vec3 g = spade::gradient(u, p);
        EXPECT_NEAR(g.x, expected.x, 1e-6f) << "n=" << n;
        EXPECT_NEAR(g.y, expected.y, 1e-6f) << "n=" << n;
        EXPECT_NEAR(g.z, expected.z, 1e-6f) << "n=" << n;
    }
}

TEST(SdfGradient, SubtractFlipsTheRemovedBranch) {
    const CsgFixture ops = make_operands();
    const SdfProgram s = make_combined(spade::SdfOp::subtract);

    int flipped = 0;
    Lcg rng(0x5eed0024u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da + db) < 1e-3f) {
            continue;
        }
        const glm::vec3 g = spade::gradient(s, p);
        if (da >= -db) {
            const glm::vec3 e = spade::gradient(ops.a, p);
            EXPECT_NEAR(g.x, e.x, 1e-6f) << "n=" << n;
        } else {
            const glm::vec3 e = -spade::gradient(ops.b, p);
            EXPECT_NEAR(g.x, e.x, 1e-6f) << "n=" << n;
            EXPECT_NEAR(g.y, e.y, 1e-6f) << "n=" << n;
            EXPECT_NEAR(g.z, e.z, 1e-6f) << "n=" << n;
            ++flipped;
        }
    }
    EXPECT_GT(flipped, 0) << "no sample exercised the subtracted branch";
}

TEST(SdfGradient, SmoothUnionBlendsBothBranchesConvexly) {
    const CsgFixture ops = make_operands();
    const float k = 0.5f;
    const SdfProgram smooth = make_combined(spade::SdfOp::smooth_union, k);

    int blended = 0;
    Lcg rng(0x5eed0025u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da - db) >= 0.5f * k) {
            continue;
        }
        const float h = glm::clamp(0.5f + 0.5f * (db - da) / k, 0.0f, 1.0f);
        const glm::vec3 expected =
            spade::gradient(ops.b, p) + (spade::gradient(ops.a, p) - spade::gradient(ops.b, p)) * h;
        const glm::vec3 g = spade::gradient(smooth, p);
        EXPECT_NEAR(g.x, expected.x, 1e-6f) << "n=" << n;
        EXPECT_NEAR(g.y, expected.y, 1e-6f) << "n=" << n;
        EXPECT_NEAR(g.z, expected.z, 1e-6f) << "n=" << n;
        // A convex blend of two unit vectors can only get shorter.
        EXPECT_LE(glm::length(g), 1.0f + kGradTol) << "n=" << n;
        ++blended;
    }
    EXPECT_GT(blended, 0);
}

// ===========================================================================
// nearest_leaf_node() -- S7a Task R8 fix round 1. Per-leaf material
// resolution needs to know WHICH leaf primitive owns the returned distance;
// this mirrors combine_distance()/combine_gradient()'s own branch selection
// exactly, checked independently (never by calling nearest_leaf_node() and
// comparing it to itself) the same way SelectingOpsCarryTheSelectedBranch-
// Gradient/SubtractFlipsTheRemovedBranch/SmoothUnionBlendsBothBranchesConvexly
// above check gradient() -- node 0 is always the sphere (ops.a), node 1 the
// box (ops.b), per make_combined()'s own authoring order.
// ===========================================================================

TEST(SdfLeaf, SingleLeafProgramAlwaysReturnsItsOwnNodeIndex) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const SdfProgram prog = sdf_of(b);
    Lcg rng(0x5eed0026u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        EXPECT_EQ(spade::nearest_leaf_node(prog, p), 0u) << "n=" << n;
    }
}

TEST(SdfLeaf, UnionPicksTheCloserOperandsLeafNode) {
    const CsgFixture ops = make_operands();
    const SdfProgram u = make_combined(spade::SdfOp::union_);

    int sphere_wins = 0, box_wins = 0;
    Lcg rng(0x5eed0027u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da - db) < 1e-3f) {
            continue;  // on the crease either branch is admissible
        }
        const uint32_t expected = da <= db ? 0u : 1u;
        EXPECT_EQ(spade::nearest_leaf_node(u, p), expected) << "n=" << n;
        expected == 0u ? ++sphere_wins : ++box_wins;
    }
    EXPECT_GT(sphere_wins, 0) << "no sample exercised the sphere's own leaf -- test is vacuous";
    EXPECT_GT(box_wins, 0) << "no sample exercised the box's own leaf -- test is vacuous";
}

TEST(SdfLeaf, IntersectPicksTheFartherOperandsLeafNode) {
    const CsgFixture ops = make_operands();
    const SdfProgram i = make_combined(spade::SdfOp::intersect);

    int flipped = 0;
    Lcg rng(0x5eed0028u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da - db) < 1e-3f) {
            continue;
        }
        const uint32_t expected = da >= db ? 0u : 1u;
        EXPECT_EQ(spade::nearest_leaf_node(i, p), expected) << "n=" << n;
        if (expected == 1u) {
            ++flipped;
        }
    }
    EXPECT_GT(flipped, 0) << "no sample exercised the box's own leaf -- test is vacuous";
}

TEST(SdfLeaf, SubtractPicksTheSubtractedLeafNodeOnItsOwnSideOfTheBoundary) {
    const CsgFixture ops = make_operands();
    const SdfProgram s = make_combined(spade::SdfOp::subtract);

    int flipped = 0;
    Lcg rng(0x5eed0029u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        if (std::abs(da + db) < 1e-3f) {
            continue;  // on the crease either branch is admissible
        }
        // Postfix "a b subtract" = max(a, -b) -- node 1 (the box, `b`) is the
        // SUBTRACTED operand, and its leaf wins exactly where combine_distance
        // keeps its (negated) branch: da < -db.
        const uint32_t expected = da >= -db ? 0u : 1u;
        EXPECT_EQ(spade::nearest_leaf_node(s, p), expected) << "n=" << n;
        if (expected == 1u) {
            ++flipped;
        }
    }
    EXPECT_GT(flipped, 0) << "no sample exercised the subtracted branch's own leaf -- test is vacuous";
}

TEST(SdfLeaf, SmoothUnionPicksTheLargerBlendWeightsLeafNode) {
    const CsgFixture ops = make_operands();
    const float k = 0.5f;
    const SdfProgram smooth = make_combined(spade::SdfOp::smooth_union, k);

    int blended = 0;
    Lcg rng(0x5eed002au);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const float da = spade::eval(ops.a, p);
        const float db = spade::eval(ops.b, p);
        const float h = glm::clamp(0.5f + 0.5f * (db - da) / k, 0.0f, 1.0f);
        if (std::abs(h - 0.5f) < 1e-3f) {
            continue;  // near-tie, either branch is admissible
        }
        const uint32_t expected = h >= 0.5f ? 0u : 1u;
        EXPECT_EQ(spade::nearest_leaf_node(smooth, p), expected) << "n=" << n;
        if (std::abs(da - db) < 0.5f * k) {
            ++blended;
        }
    }
    EXPECT_GT(blended, 0) << "no sample landed in the blend band -- test is vacuous";
}

TEST(SdfLeaf, EmptyProgramReturnsTheNoLeafSentinel) {
    const SdfProgram empty;
    EXPECT_EQ(spade::nearest_leaf_node(empty, glm::vec3(0.0f)), spade::kNoSdfLeaf);
}

// ===========================================================================
// eval / sample agreement and the empty program
// ===========================================================================

TEST(SdfEval, SampleDistanceIsBitIdenticalToEval) {
    const SdfProgram prog = make_combined(spade::SdfOp::smooth_union, 0.4f);
    Lcg rng(0x5eed0030u);
    for (int n = 0; n < kSamples; ++n) {
        const glm::vec3 p = rng.point(2.0f);
        const spade::SdfSample s = spade::sample(prog, p);
        EXPECT_EQ(s.distance, spade::eval(prog, p)) << "n=" << n;
        const glm::vec3 g = spade::gradient(prog, p);
        EXPECT_EQ(s.gradient.x, g.x) << "n=" << n;
    }
}

TEST(SdfEval, EmptyProgramIsFarAwayWithZeroGradient) {
    const SdfProgram prog = sdf_of(base_builder());  // no SDF nodes added
    EXPECT_TRUE(prog.empty());
    EXPECT_EQ(spade::eval(prog, glm::vec3(1.0f, 2.0f, 3.0f)), spade::kSdfEmptyDistance);
    EXPECT_EQ(spade::gradient(prog, glm::vec3(1.0f, 2.0f, 3.0f)), glm::vec3(0.0f));
}

// ===========================================================================
// The gate: torus + two posts (the shape D3 calls out as the reason for
// analytic CSG -- "race gates are box/torus compositions, exact geometry,
// zero mesh error").
// ===========================================================================

namespace {

constexpr float kGateMajor = 1.5f;
constexpr float kGateMinor = 0.15f;
constexpr float kGateHeight = 2.0f;   // ring centre height
constexpr float kPostHalf = 0.1f;

SdfProgram make_gate() {
    // Ring: local hole axis is +Y, so a +90 degree turn about X points it down
    // +Z -- a drone flies through along Z, and the ring lies in the world XY
    // plane. The two posts run from the ground up to the ring's sides.
    const SdfPose ring{.position = glm::vec3(0.0f, kGateHeight, 0.0f),
                       .rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
                       .scale = 1.0f};
    const glm::vec3 post_half(kPostHalf, kGateHeight * 0.5f, kPostHalf);
    const SdfPose left{.position = glm::vec3(-kGateMajor, kGateHeight * 0.5f, 0.0f),
                       .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                       .scale = 1.0f};
    const SdfPose right{.position = glm::vec3(kGateMajor, kGateHeight * 0.5f, 0.0f),
                        .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                        .scale = 1.0f};

    WorldBuilder b = base_builder();
    b.name("gate")
        .spawn("start", glm::vec3(0.0f, kGateHeight, -5.0f))
        .torus(kGateMajor, kGateMinor, ring)
        .box(post_half, left)
        .union_()
        .box(post_half, right)
        .union_();
    return sdf_of(b);
}

}  // namespace

TEST(SdfGate, OpeningIsFreeSpaceAndStructureIsSolid) {
    const SdfProgram gate = make_gate();

    // Dead centre of the opening: clear by (major - minor).
    EXPECT_NEAR(spade::eval(gate, glm::vec3(0.0f, kGateHeight, 0.0f)), kGateMajor - kGateMinor,
                kDistTol);

    // On the ring's tube centre-line: inside the wall by the tube radius.
    EXPECT_NEAR(spade::eval(gate, glm::vec3(kGateMajor, kGateHeight, 0.0f)), -kGateMinor, kDistTol);
    EXPECT_NEAR(spade::eval(gate, glm::vec3(0.0f, kGateHeight + kGateMajor, 0.0f)), -kGateMinor,
                kDistTol);

    // Inside a post.
    EXPECT_LT(spade::eval(gate, glm::vec3(kGateMajor, 0.5f, 0.0f)), 0.0f);
    EXPECT_LT(spade::eval(gate, glm::vec3(-kGateMajor, 0.5f, 0.0f)), 0.0f);
}

TEST(SdfGate, FlightLineThroughTheHoleStaysPositive) {
    const SdfProgram gate = make_gate();
    for (int i = -30; i <= 30; ++i) {
        const float z = static_cast<float>(i) * 0.1f;
        const glm::vec3 p(0.0f, kGateHeight, z);
        EXPECT_GT(spade::eval(gate, p), 0.5f) << "z=" << z;
        // Off-centre but still inside the opening.
        EXPECT_GT(spade::eval(gate, p + glm::vec3(0.5f, 0.0f, 0.0f)), 0.0f) << "z=" << z;
    }
}

TEST(SdfGate, GrazingTheRingCrossesZero) {
    const SdfProgram gate = make_gate();
    // Walking outward through the ring's tube must change sign twice.
    const float y = kGateHeight;
    EXPECT_GT(spade::eval(gate, glm::vec3(kGateMajor - 2.0f * kGateMinor, y, 0.0f)), 0.0f);
    EXPECT_LT(spade::eval(gate, glm::vec3(kGateMajor, y, 0.0f)), 0.0f);
    EXPECT_GT(spade::eval(gate, glm::vec3(kGateMajor + 2.0f * kGateMinor, y, 0.0f)), 0.0f);
}

TEST(SdfGate, ProgramShapeIsPostfixAndShallow) {
    const SdfProgram gate = make_gate();
    ASSERT_EQ(gate.nodes.size(), 5u);  // torus, box, union, box, union
    const spade::Result<uint32_t> depth = gate.validate();
    ASSERT_TRUE(depth.has_value()) << depth.error().context;
    EXPECT_EQ(*depth, 2u);
}

// ===========================================================================
// Program validation
// ===========================================================================

TEST(SdfValidate, ReportsPeakStackDepth) {
    WorldBuilder b = base_builder();
    for (int i = 0; i < 5; ++i) {
        b.sphere(1.0f);
    }
    for (int i = 0; i < 4; ++i) {
        b.union_();
    }
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const spade::Result<uint32_t> depth = world->sdf.validate();
    ASSERT_TRUE(depth.has_value());
    EXPECT_EQ(*depth, 5u);
}

TEST(SdfValidate, DepthAtTheLimitIsAccepted) {
    WorldBuilder b = base_builder();
    for (uint32_t i = 0; i < spade::kMaxSdfDepth; ++i) {
        b.sphere(1.0f);
    }
    for (uint32_t i = 0; i < spade::kMaxSdfDepth - 1; ++i) {
        b.union_();
    }
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const spade::Result<uint32_t> depth = world->sdf.validate();
    ASSERT_TRUE(depth.has_value());
    EXPECT_EQ(*depth, spade::kMaxSdfDepth);
}

TEST(SdfValidate, DepthOverTheLimitIsCapacityExceeded) {
    WorldBuilder b = base_builder();
    for (uint32_t i = 0; i < spade::kMaxSdfDepth + 1; ++i) {
        b.sphere(1.0f);
    }
    for (uint32_t i = 0; i < spade::kMaxSdfDepth; ++i) {
        b.union_();  // well-formed postfix -- depth is the ONLY defect
    }
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::capacity_exceeded) << world.error().context;
}

TEST(SdfValidate, OperatorWithoutTwoOperandsIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).union_();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(SdfValidate, LeftoverOperandsAreInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).sphere(2.0f);  // never combined
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(SdfValidate, HandBuiltProgramWithBadTransformIndexIsInvalid) {
    SdfProgram prog;
    prog.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    node.transform = 7;  // out of range
    node.params = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    prog.nodes.push_back(node);
    const spade::Result<uint32_t> r = prog.validate();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument);
}

TEST(SdfValidate, HandBuiltProgramWithNonUnitPlaneNormalIsInvalid) {
    SdfProgram prog;
    prog.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::plane);
    node.params = glm::vec4(0.0f, 3.0f, 0.0f, 0.0f);  // |n| == 3
    prog.nodes.push_back(node);
    const spade::Result<uint32_t> r = prog.validate();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument);
}

TEST(SdfValidate, HandBuiltProgramWithNonFiniteParamsIsInvalid) {
    SdfProgram prog;
    prog.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    node.params = glm::vec4(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f);
    prog.nodes.push_back(node);
    const spade::Result<uint32_t> r = prog.validate();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument);
}

TEST(SdfValidate, NegativeSmoothUnionRadiusIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).sphere(1.0f).smooth_union(-0.5f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(SdfValidate, EmptyProgramValidatesToZeroDepth) {
    const SdfProgram prog;
    const spade::Result<uint32_t> r = prog.validate();
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 0u);
}

// ---------------------------------------------------------------------------
// node_materials (schema v2, PA-2): a parallel, host-only array. validate()
// only checks its LENGTH -- whether an index is in range depends on
// WorldDesc::materials, which SdfProgram does not have, so that half lives in
// validate_world_desc() (WorldBuilderValidation section below).
// ---------------------------------------------------------------------------

TEST(SdfValidate, EmptyNodeMaterialsIsAccepted) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).sphere(1.0f).union_();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    EXPECT_TRUE(world->sdf.node_materials.empty());
    EXPECT_TRUE(world->sdf.validate().has_value());
}

TEST(SdfValidate, NodeMaterialsWrongLengthIsInvalid) {
    SdfProgram prog;
    prog.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    node.params = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    prog.nodes.push_back(node);
    prog.nodes.push_back(node);
    // Two nodes; node_materials of length 1 is neither 0 nor nodes.size().
    prog.node_materials = {0u};
    const spade::Result<uint32_t> r = prog.validate();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument);
    EXPECT_NE(r.error().context.find("node_materials"), std::string::npos) << r.error().context;
}

TEST(SdfValidate, FullLengthNodeMaterialsIsAccepted) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).sphere(1.0f).union_();  // 3 nodes: sphere, sphere, union
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    SdfProgram prog = world->sdf;
    ASSERT_EQ(prog.nodes.size(), 3u);
    prog.node_materials = {0u, 1u, 0u};  // one per node, including the op node
    const spade::Result<uint32_t> r = prog.validate();
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().context);
}

// ===========================================================================
// WorldBuilder validation
// ===========================================================================

TEST(WorldBuilderValidation, ZeroCapacityIsInvalid) {
    const Capacities full{.bodies = 4, .force_elements = 8, .sensors = 2, .contacts = 16};

    Capacities cases[4] = {full, full, full, full};
    cases[0].bodies = 0;
    cases[1].force_elements = 0;
    cases[2].sensors = 0;
    cases[3].contacts = 0;

    for (const Capacities& caps : cases) {
        WorldBuilder b;
        b.name("w").capacities(caps).sphere(1.0f);
        const spade::Result<WorldDesc> world = b.build();
        ASSERT_FALSE(world.has_value());
        EXPECT_EQ(world.error().code, Code::invalid_argument) << world.error().context;
    }
}

TEST(WorldBuilderValidation, DuplicateSpawnNameIsInvalid) {
    WorldBuilder b = base_builder();
    b.spawn("start", glm::vec3(0.0f))
        .spawn("pit", glm::vec3(1.0f, 0.0f, 0.0f))
        .spawn("start", glm::vec3(2.0f, 0.0f, 0.0f));
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("start"), std::string::npos) << world.error().context;
}

TEST(WorldBuilderValidation, UnnamedSpawnIsInvalid) {
    WorldBuilder b = base_builder();
    b.spawn("", glm::vec3(0.0f));
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldBuilderValidation, DegeneratePlaneNormalIsInvalid) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f), 1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldBuilderValidation, NonPositivePoseScaleIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f, SdfPose{.position = glm::vec3(0.0f),
                           .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                           .scale = 0.0f});
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldBuilderValidation, NegativeRadiusIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(-1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldBuilderValidation, NonFiniteEnvironmentIsInvalid) {
    Environment env;
    env.gravity.y = std::numeric_limits<float>::infinity();
    WorldBuilder b = base_builder();
    b.environment(env).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldBuilderValidation, FirstErrorSurvivesLaterAdds) {
    WorldBuilder b = base_builder();
    b.plane(glm::vec3(0.0f), 1.0f)   // degenerate: recorded here
        .sphere(1.0f)
        .union_();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_NE(world.error().context.find("plane normal"), std::string::npos)
        << world.error().context;
}

// ===========================================================================
// Materials, lighting, props (schema v2, S7a task W1)
// ===========================================================================

using spade::MaterialDesc;
using spade::MaterialShading;
using spade::LightingDesc;
using spade::PropDesc;

TEST(WorldMaterials, BuildWithNoMaterialCallGetsOneDefaultMaterial) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->materials.size(), 1u);
    EXPECT_EQ(world->materials[0].shading, MaterialShading::lambert);
    EXPECT_TRUE(world->props.empty());
}

TEST(WorldMaterials, MaterialCallsAppendInOrder) {
    WorldBuilder b = base_builder();
    MaterialDesc red;
    red.name = "red";
    red.base_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    red.shading = MaterialShading::unlit;
    MaterialDesc glow;
    glow.name = "glow";
    glow.shading = MaterialShading::emissive;
    b.material(red).material(glow).sphere(1.0f);

    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->materials.size(), 2u);
    EXPECT_EQ(world->materials[0].name, "red");
    EXPECT_EQ(world->materials[0].shading, MaterialShading::unlit);
    EXPECT_EQ(world->materials[1].name, "glow");
    EXPECT_EQ(world->materials[1].shading, MaterialShading::emissive);
}

TEST(WorldMaterials, MaterialForLastNodeSetsOnlyThatNodeAndDefaultsEarlierOnesToZero) {
    WorldBuilder b = base_builder();
    MaterialDesc primary;
    primary.name = "primary";
    MaterialDesc extra;
    extra.name = "extra";
    b.material(primary)   // index 0
        .material(extra)  // index 1
        .sphere(1.0f)
        .sphere(1.0f)
        .material_for_last_node(1)
        .union_();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->materials.size(), 2u);
    ASSERT_EQ(world->sdf.nodes.size(), 3u);
    ASSERT_EQ(world->sdf.node_materials.size(), 3u);
    EXPECT_EQ(world->sdf.node_materials[0], 0u);  // first sphere: default ("primary")
    EXPECT_EQ(world->sdf.node_materials[1], 1u);  // second sphere: set explicitly ("extra")
    EXPECT_EQ(world->sdf.node_materials[2], 0u);  // union node: default
}

TEST(WorldMaterials, MaterialForLastNodeWithNoNodeYetIsInvalid) {
    WorldBuilder b = base_builder();
    b.material_for_last_node(0).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldMaterials, NodeMaterialIndexOutOfRangeIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).material_for_last_node(5);  // only material 0 (the default) exists
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("material index"), std::string::npos)
        << world.error().context;
}

TEST(WorldMaterials, UnknownShadingValueIsInvalid) {
    WorldBuilder b = base_builder();
    MaterialDesc weird;
    weird.shading = static_cast<MaterialShading>(99);
    b.material(weird).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("shading"), std::string::npos) << world.error().context;
}

// W1 fix round: name must be non-empty, the same rule visual_refs and
// PropDesc::mesh_ref already carry (an empty string names nothing).
TEST(WorldMaterials, EmptyNameIsInvalid) {
    WorldBuilder b = base_builder();
    MaterialDesc nameless;
    nameless.name = "";
    b.material(nameless).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("empty name"), std::string::npos) << world.error().context;
}

TEST(WorldLighting, DefaultLightingIsFinite) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const LightingDesc& light = world->lighting;
    EXPECT_TRUE(std::isfinite(light.sun_direction.x));
    EXPECT_TRUE(std::isfinite(light.sun_intensity));
    EXPECT_TRUE(std::isfinite(light.sky_zenith.x));
    EXPECT_TRUE(std::isfinite(light.sky_horizon.x));
}

TEST(WorldLighting, LightingCallReplacesTheDefault) {
    WorldBuilder b = base_builder();
    LightingDesc light;
    light.sun_direction = glm::vec3(1.0f, 0.0f, 0.0f);
    light.sun_intensity = 3.5f;
    b.lighting(light).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    EXPECT_EQ(world->lighting.sun_direction, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(world->lighting.sun_intensity, 3.5f);
}

TEST(WorldLighting, NonFiniteLightingIsInvalid) {
    WorldBuilder b = base_builder();
    LightingDesc light;
    light.sun_intensity = std::numeric_limits<float>::infinity();
    b.lighting(light).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("lighting"), std::string::npos) << world.error().context;
}

// W1 fix round: sun_direction is documented as "need not be pre-normalized --
// R6 normalizes it" (builder.hpp), which makes a ZERO vector a reachable,
// validating world that hands R6 a NaN the instant it normalizes -- finite
// alone does not catch it (0 is finite). Same shape as
// WorldBuilderValidation.DegeneratePlaneNormalIsInvalid.
TEST(WorldLighting, ZeroSunDirectionIsInvalid) {
    WorldBuilder b = base_builder();
    LightingDesc light;
    light.sun_direction = glm::vec3(0.0f);
    b.lighting(light).sphere(1.0f);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("sun_direction"), std::string::npos)
        << world.error().context;
}

TEST(WorldProps, PropIsKeptWithItsPoseAndMaterial) {
    WorldBuilder b = base_builder();
    SdfPose pose;
    pose.position = glm::vec3(1.0f, 2.0f, 3.0f);
    b.sphere(1.0f).prop("mesh:cone", pose, 0);
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->props.size(), 1u);
    EXPECT_EQ(world->props[0].mesh_ref, "mesh:cone");
    EXPECT_EQ(world->props[0].pose.position, glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(world->props[0].material, 0u);
}

TEST(WorldProps, EmptyMeshRefIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).prop("", SdfPose{});
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldProps, NonPositivePoseScaleIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).prop("mesh:cone", SdfPose{.position = glm::vec3(0.0f),
                                             .rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                             .scale = 0.0f});
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
}

TEST(WorldProps, MaterialIndexOutOfRangeIsInvalid) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).prop("mesh:cone", SdfPose{}, 7);  // only material 0 exists
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_FALSE(world.has_value());
    EXPECT_EQ(world.error().code, Code::invalid_argument);
    EXPECT_NE(world.error().context.find("material index"), std::string::npos)
        << world.error().context;
}

// ===========================================================================
// WorldDesc product
// ===========================================================================

TEST(WorldDescProduct, CarriesNameEnvironmentCapacitiesAndSpawns) {
    Environment env;
    env.gravity = glm::vec3(0.0f, -3.71f, 0.0f);  // Mars, because it must round-trip verbatim
    env.wind = glm::vec3(2.0f, 0.0f, -1.0f);
    env.air_density = 0.02f;
    env.temperature_k = 210.0f;
    env.seed = 0xDEADBEEFCAFEull;
    const Capacities caps{.bodies = 8, .force_elements = 32, .sensors = 4, .contacts = 64};

    WorldBuilder b;
    b.name("mars-loop")
        .environment(env)
        .capacities(caps)
        .spawn("start", glm::vec3(0.0f, 1.0f, 0.0f),
               glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)))
        .spawn("pit", glm::vec3(5.0f, 0.5f, 0.0f))
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);

    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    EXPECT_EQ(world->name, "mars-loop");
    EXPECT_EQ(world->environment.gravity, env.gravity);
    EXPECT_EQ(world->environment.wind, env.wind);
    EXPECT_FLOAT_EQ(world->environment.air_density, env.air_density);
    EXPECT_FLOAT_EQ(world->environment.temperature_k, env.temperature_k);
    EXPECT_EQ(world->environment.seed, env.seed);
    EXPECT_EQ(world->capacities.bodies, caps.bodies);
    EXPECT_EQ(world->capacities.force_elements, caps.force_elements);
    EXPECT_EQ(world->capacities.sensors, caps.sensors);
    EXPECT_EQ(world->capacities.contacts, caps.contacts);
    ASSERT_EQ(world->spawns.size(), 2u);

    const SpawnPoint* start = world->find_spawn("start");
    ASSERT_NE(start, nullptr);
    EXPECT_EQ(start->position, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_NEAR(glm::length(start->orientation), 1.0f, 1e-6f);
    EXPECT_NE(world->find_spawn("pit"), nullptr);
    EXPECT_EQ(world->find_spawn("nowhere"), nullptr);
}

TEST(WorldDescProduct, DefaultEnvironmentIsEarthYUp) {
    WorldBuilder b = base_builder();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    EXPECT_FLOAT_EQ(world->environment.gravity.x, 0.0f);
    EXPECT_FLOAT_EQ(world->environment.gravity.y, -9.80665f);
    EXPECT_FLOAT_EQ(world->environment.gravity.z, 0.0f);
    EXPECT_FLOAT_EQ(world->environment.air_density, 1.225f);
}

TEST(WorldDescProduct, IdentityPosesShareTransformZero) {
    WorldBuilder b = base_builder();
    b.sphere(1.0f).sphere(2.0f).union_();
    const spade::Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    ASSERT_EQ(world->sdf.transforms.size(), 1u);
    EXPECT_EQ(world->sdf.nodes[0].transform, 0u);
    EXPECT_EQ(world->sdf.nodes[1].transform, 0u);
}

// ===========================================================================
// POD layout -- these travel to a GPU buffer at S6.
// ===========================================================================

TEST(SdfLayout, NodeAndTransformArePodsOfPinnedSize) {
    EXPECT_EQ(sizeof(spade::SdfNode), 32u);
    EXPECT_EQ(sizeof(spade::SdfTransform), 80u);
    EXPECT_TRUE(std::is_trivially_copyable_v<spade::SdfNode>);
    EXPECT_TRUE(std::is_trivially_copyable_v<spade::SdfTransform>);
}
