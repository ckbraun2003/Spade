// The sandbox's screen ray and unit meshes (screen_ray.hpp, unit_meshes.hpp),
// asserted with no window: a click's ray, and the box, sphere and cylinder
// every view scales. Moved from test_sandbox_builder.cpp with the code when
// the builder mode left the sandbox (INT-9); the cases are unchanged.
//
// EXPECTATIONS ARE HAND-WRITTEN LITERALS, NOT A SECOND EXPRESSION OF THE CODE
// UNDER TEST: an identity camera looks along -Z, so the expected directions
// are arithmetic a reader can do on paper.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/screen_ray.hpp"
#include "../sandbox/unit_meshes.hpp"

namespace {

using namespace spade::sandbox;

// A camera at the origin looking along -Z, level. Used for the axis tests,
// where an identity orientation makes the expected direction obvious.
spade::render::Camera level_camera() {
    spade::render::Camera c;
    c.position = glm::vec3(0.0f, 0.0f, 0.0f);
    c.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);   // identity: forward is -Z
    return c;
}

// THE PIXEL WHOSE CENTRE IS THE FRAME CENTRE, WHICH IS NOT w/2.
//
// ⚠ ray_from_screen samples the pixel CENTRE -- it adds 0.5 to the index, the
// standard convention and the one the rasteriser uses. So for a 400-wide
// frame, the ray through the exact middle comes from index 199.5, not 200.
// Passing 200 offsets the ray by half a pixel, which is 0.0029 in direction
// and about 0.02 m on the ground at 10 m. THREE CASES FAILED ON EXACTLY THAT
// AND THE CODE WAS RIGHT EACH TIME -- the fixture was asking the wrong pixel.
// ⭐ Kept as a named helper rather than a magic 199.5 so the next reader meets
// the convention instead of rediscovering it.
[[nodiscard]] float centre_of(uint32_t extent) {
    return static_cast<float>(extent) * 0.5f - 0.5f;
}

}  // namespace

// ---------------------------------------------------------------------------
// The screen ray
// ---------------------------------------------------------------------------

TEST(SandboxScreenRay, CentrePixelLooksAlongTheCameraForward) {
    const Ray r = ray_from_screen(level_camera(), centre_of(400u), centre_of(200u), 400u, 200u);
    // Identity orientation, so forward is world -Z. Centre pixel, so no lateral
    // component at all.
    EXPECT_NEAR(r.direction.x, 0.0f, 1e-3f);
    EXPECT_NEAR(r.direction.y, 0.0f, 1e-3f);
    EXPECT_NEAR(r.direction.z, -1.0f, 1e-3f);
}

// THE Y FLIP, ISOLATED. Screen y grows DOWNWARD and NDC y grows UPWARD, so a
// click ABOVE the centre must produce a ray pointing UP. Getting this backwards
// gives a picker that is correct on one axis and mirrored on the other, which
// reads as unreliability rather than as a sign error.
TEST(SandboxScreenRay, ClickingAboveCentreAimsUpwardAndBelowAimsDown) {
    const spade::render::Camera c = level_camera();
    const Ray up = ray_from_screen(c, 200.0f, 20.0f, 400u, 200u);
    const Ray down = ray_from_screen(c, 200.0f, 180.0f, 400u, 200u);
    EXPECT_GT(up.direction.y, 0.05f);
    EXPECT_LT(down.direction.y, -0.05f);
}

TEST(SandboxScreenRay, ClickingRightOfCentreAimsRight) {
    const Ray r = ray_from_screen(level_camera(), 380.0f, 100.0f, 400u, 200u);
    EXPECT_GT(r.direction.x, 0.05f);
}

// ---------------------------------------------------------------------------
// The unit meshes
// ---------------------------------------------------------------------------

TEST(SandboxUnitMesh, BoxHasTwelveTrianglesAndUnitNormals) {
    const spade::render::MeshData m = make_box_mesh();
    EXPECT_EQ(m.indices.size(), 36u);       // 6 faces * 2 triangles * 3
    EXPECT_EQ(m.positions.size(), 36u);     // flat shading duplicates vertices
    ASSERT_EQ(m.normals.size(), m.positions.size());
    for (const glm::vec3& n : m.normals) {
        EXPECT_NEAR(glm::length(n), 1.0f, 1e-4f);
    }
    ASSERT_EQ(m.submesh_first_index.size(), 1u);
    EXPECT_EQ(m.submesh_index_count[0], 36u);
}

// EVERY PRIMITIVE SPANS [-0.5, 0.5] ON EVERY AXIS. That is what makes `scale`
// mean metres, and it is asserted rather than assumed because a primitive
// built at unit RADIUS instead of unit DIAMETER is off by 2x and looks
// perfectly reasonable on screen.
TEST(SandboxUnitMesh, EveryPrimitiveIsAUnitShape) {
    const spade::render::MeshData meshes[3] = {make_box_mesh(), make_sphere_mesh(),
                                               make_cylinder_mesh()};
    for (const spade::render::MeshData& m : meshes) {
        ASSERT_FALSE(m.positions.empty());
        glm::vec3 lo(1e9f), hi(-1e9f);
        for (const glm::vec3& p : m.positions) {
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        EXPECT_NEAR(hi.x - lo.x, 1.0f, 0.02f);
        EXPECT_NEAR(hi.y - lo.y, 1.0f, 0.02f);
        EXPECT_NEAR(hi.z - lo.z, 1.0f, 0.02f);
    }
}

// EVERY TRIANGLE FACES OUT. Both shading paths light by the stored normal and
// the CPU path culls by screen winding, so an inward-wound primitive is lit
// from the wrong side and drawn from the inside. The sphere and cylinder were
// wound inward until 2026-10-02, and the sphere's pole caps were zero-area
// triangles with NaN normals; nothing noticed, because the GL path's sun was
// also inverted and the two errors cancelled. A degenerate triangle fails here
// too: its winding is zero and its normal is not a number.
TEST(SandboxUnitMesh, EveryTriangleFacesOutward) {
    // The last two are the sized meshes the drone's parts use (a flat body
    // box and a 1 cm rotor disc), so the size parameters keep the winding.
    const spade::render::MeshData meshes[5] = {make_box_mesh(), make_sphere_mesh(), make_cylinder_mesh(),
                                               make_box_mesh(glm::vec3(0.06f, 0.025f, 0.06f)),
                                               make_cylinder_mesh(24u, 0.12f, 0.005f)};
    for (int k = 0; k < 5; ++k) {
        const spade::render::MeshData& m = meshes[k];
        ASSERT_FALSE(m.indices.empty());
        for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const glm::vec3 a = m.positions[m.indices[t]];
            const glm::vec3 b = m.positions[m.indices[t + 1]];
            const glm::vec3 c = m.positions[m.indices[t + 2]];
            const glm::vec3 centroid = (a + b + c) / 3.0f;  // every primitive is centred on the origin
            ASSERT_GT(glm::dot(glm::cross(b - a, c - a), centroid), 0.0f) << "mesh " << k << " triangle " << t / 3;
            ASSERT_GT(glm::dot(m.normals[m.indices[t]], centroid), 0.0f) << "mesh " << k << " triangle " << t / 3;
        }
    }
}
