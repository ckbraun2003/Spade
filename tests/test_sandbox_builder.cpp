// The builder's behaviour, asserted with no window anywhere in scope.
//
// WHY THIS FILE IS THE POINT OF THE BUILDER'S DESIGN. Placing, picking and
// dragging feel like window code because that is where the mouse is, and a
// builder that puts them in the window loop can only ever be verified by a
// human saying "it looked right when I dragged it". That is not a claim
// anyone can re-run. Every behaviour below is decided by a pure function in
// builder_scene.hpp, so it is checkable here against an answer computed BY
// HAND.
//
// EXPECTATIONS ARE HAND-WRITTEN LITERALS, NOT A SECOND EXPRESSION OF THE CODE
// UNDER TEST. This suite already shipped one oracle that was a byte-identical
// restatement of its subject (ledger row L301) and could therefore only ever
// agree with itself. The fixtures here are chosen so the right answer is
// arithmetic a reader can do on paper: a camera at (0,5,5) looking at the
// origin meets the ground plane at EXACTLY the origin, so the numbers below
// are 0 and 0.5 rather than whatever the implementation returns.

#include <gtest/gtest.h>

#include <cmath>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/builder_scene.hpp"

namespace {

using namespace spade::sandbox;

// A camera at (0,5,5) aimed at the origin. Its centre ray meets y=0 at the
// origin exactly -- the whole reason this pose was chosen.
spade::render::Camera diagonal_camera() {
    spade::render::Camera c;
    c.position = glm::vec3(0.0f, 5.0f, 5.0f);
    c.orientation = glm::quatLookAt(glm::normalize(glm::vec3(0.0f, -5.0f, -5.0f)),
                                    glm::vec3(0.0f, 1.0f, 0.0f));
    return c;
}

// A camera at the origin looking along -Z, level. Used for the axis tests,
// where an identity orientation makes the expected direction obvious.
spade::render::Camera level_camera() {
    spade::render::Camera c;
    c.position = glm::vec3(0.0f, 0.0f, 0.0f);
    c.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);   // identity: forward is -Z
    return c;
}

// A camera that genuinely looks ABOVE the horizon.
//
// ⚠⚠ THIS EXISTS BECAUSE MY FIRST ATTEMPT AT THE "off the ground plane" CASE
// WAS BUILT ON A FALSE PREMISE AND THE TEST CAUGHT IT. I clicked the top row
// of the frame with diagonal_camera() and expected a miss. That camera is
// pitched 45 degrees DOWN and the vertical half-fov is 30 degrees, so NO PIXEL
// IN THAT FRAME LOOKS ABOVE THE HORIZON -- the top row still hits the ground,
// 15 degrees below horizontal. The case was asserting something the fixture
// made impossible. ⭐ A test whose fixture cannot produce the condition under
// test passes or fails for reasons unrelated to the code.
spade::render::Camera sky_camera() {
    spade::render::Camera c;
    c.position = glm::vec3(0.0f, 2.0f, 5.0f);
    c.orientation = glm::quatLookAt(glm::normalize(glm::vec3(0.0f, 1.0f, -1.0f)),
                                    glm::vec3(0.0f, 1.0f, 0.0f));
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

FrameInput click_at(float x, float y) {
    FrameInput in;
    in.mouse_x = x;
    in.mouse_y = y;
    in.left_click = true;
    in.left_down = true;
    return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// Ray construction -- the two sign errors that make a picker "feel flaky"
// ---------------------------------------------------------------------------

TEST(SandboxBuilderRay, CentrePixelLooksAlongTheCameraForward) {
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
TEST(SandboxBuilderRay, ClickingAboveCentreAimsUpwardAndBelowAimsDown) {
    const spade::render::Camera c = level_camera();
    const Ray up = ray_from_screen(c, 200.0f, 20.0f, 400u, 200u);
    const Ray down = ray_from_screen(c, 200.0f, 180.0f, 400u, 200u);
    EXPECT_GT(up.direction.y, 0.05f);
    EXPECT_LT(down.direction.y, -0.05f);
}

TEST(SandboxBuilderRay, ClickingRightOfCentreAimsRight) {
    const Ray r = ray_from_screen(level_camera(), 380.0f, 100.0f, 400u, 200u);
    EXPECT_GT(r.direction.x, 0.05f);
}

// ---------------------------------------------------------------------------
// The ground hit -- and the case that puts objects behind you
// ---------------------------------------------------------------------------

TEST(SandboxBuilderGround, DiagonalCameraCentreRayMeetsTheOrigin) {
    const Ray r = ray_from_screen(diagonal_camera(), centre_of(400u), centre_of(200u), 400u, 200u);
    const std::optional<glm::vec3> hit = ray_ground_hit(r, 0.0f);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->x, 0.0f, 1e-3f);
    EXPECT_NEAR(hit->y, 0.0f, 1e-3f);
    EXPECT_NEAR(hit->z, 0.0f, 1e-3f);
}

// A RAY AIMED AT THE SKY HAS A PERFECTLY GOOD ALGEBRAIC INTERSECTION BEHIND
// THE CAMERA. A placement tool that takes it drops objects behind the user
// whenever they click near the horizon, and the object is out of frame so the
// click reads as having done nothing at all.
TEST(SandboxBuilderGround, ARayPointingAwayFromThePlaneDoesNotHitIt) {
    Ray up;
    up.origin = glm::vec3(0.0f, 5.0f, 0.0f);
    up.direction = glm::normalize(glm::vec3(0.0f, 1.0f, -1.0f));
    EXPECT_FALSE(ray_ground_hit(up, 0.0f).has_value());
}

TEST(SandboxBuilderGround, ARayParallelToThePlaneDoesNotHitIt) {
    Ray flat;
    flat.origin = glm::vec3(0.0f, 5.0f, 0.0f);
    flat.direction = glm::vec3(0.0f, 0.0f, -1.0f);
    EXPECT_FALSE(ray_ground_hit(flat, 0.0f).has_value());
}

// ---------------------------------------------------------------------------
// Picking
// ---------------------------------------------------------------------------

// THE DISCRIMINATING FIXTURE, AND THE ORDER IS THE POINT: the FAR object is
// inserted FIRST, so an implementation that returns the first hit rather than
// the nearest returns index 0 and this test goes red. With the near one first,
// both implementations agree and the case proves nothing -- the coincidence
// trap this realm has a standing rule about.
TEST(SandboxBuilderPick, ReturnsTheNearestObjectNotTheFirstInTheList) {
    BuilderScene s;
    BuilderObject far_box;                                  // index 0, FAR
    far_box.position = glm::vec3(0.0f, 0.0f, -20.0f);
    BuilderObject near_box;                                 // index 1, NEAR
    near_box.position = glm::vec3(0.0f, 0.0f, -2.0f);
    s.objects.push_back(far_box);
    s.objects.push_back(near_box);

    const Ray r = ray_from_screen(level_camera(), 200.0f, 100.0f, 400u, 200u);
    EXPECT_EQ(pick_object(s, r), 1);
}

TEST(SandboxBuilderPick, MissesWhenNothingIsUnderTheRay) {
    BuilderScene s;
    BuilderObject o;
    o.position = glm::vec3(40.0f, 0.0f, -5.0f);   // well off to the side
    s.objects.push_back(o);
    const Ray r = ray_from_screen(level_camera(), 200.0f, 100.0f, 400u, 200u);
    EXPECT_EQ(pick_object(s, r), -1);
}

// A SPHERE IS NOT ITS BOUNDING BOX. A ray through a corner of the unit cube
// that contains the sphere misses the sphere, and an implementation that
// picked against an AABB would select an object the user can see they did not
// click. The two arms differ ONLY in `shape`, so the shape is what is proved.
TEST(SandboxBuilderPick, SphereIsTestedAsASphereAndNotAsItsBox) {
    const Ray corner{glm::vec3(0.49f, 0.49f, 4.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    BuilderObject box;
    box.shape = Shape::box;
    box.position = glm::vec3(0.0f);
    BuilderObject sphere;
    sphere.shape = Shape::sphere;
    sphere.position = glm::vec3(0.0f);

    EXPECT_TRUE(ray_object_hit(corner, box).has_value());
    EXPECT_FALSE(ray_object_hit(corner, sphere).has_value());
}

// Scale is FULL EXTENT, not half-extent -- the two conventions look identical
// on screen and differ by exactly 2x in every number, so one case pins it.
TEST(SandboxBuilderPick, ScaleIsFullExtent) {
    BuilderObject o;
    o.shape = Shape::box;
    o.position = glm::vec3(0.0f);
    o.scale = glm::vec3(2.0f);   // a 2 m cube spans -1 .. +1

    const Ray inside{glm::vec3(0.9f, 0.0f, 4.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    const Ray outside{glm::vec3(1.1f, 0.0f, 4.0f), glm::vec3(0.0f, 0.0f, -1.0f)};
    EXPECT_TRUE(ray_object_hit(inside, o).has_value());
    EXPECT_FALSE(ray_object_hit(outside, o).has_value());
}

// ---------------------------------------------------------------------------
// Placement
// ---------------------------------------------------------------------------

// THE HALF-HEIGHT OFFSET, ASSERTED RATHER THAN EYEBALLED. Placing an object's
// CENTRE at the ground hit buries half of it under the floor every time.
TEST(SandboxBuilderPlace, ANewObjectRestsOnTheGroundRatherThanInIt) {
    const BuilderObject o = make_object_at(Shape::box, glm::vec3(3.0f, 0.0f, -4.0f), 0);
    EXPECT_NEAR(o.position.x, 3.0f, 1e-5f);
    EXPECT_NEAR(o.position.z, -4.0f, 1e-5f);
    EXPECT_NEAR(o.position.y, 0.5f, 1e-5f);   // half of the default 1 m height
}

TEST(SandboxBuilderPlace, AClickOnEmptyGroundAddsAnObjectThere) {
    BuilderScene s;
    s.placing = true;
    const spade::render::Camera c = diagonal_camera();
    apply_builder_input(s, click_at(centre_of(400u), centre_of(200u)), c, 400u, 200u, 0.0f);

    ASSERT_EQ(s.objects.size(), 1u);
    EXPECT_EQ(s.selected, 0);
    // The centre ray of this camera meets the ground at the origin, so the new
    // object sits at (0, 0.5, 0) -- computed by hand, not read back.
    EXPECT_NEAR(s.objects[0].position.x, 0.0f, 1e-3f);
    EXPECT_NEAR(s.objects[0].position.y, 0.5f, 1e-3f);
    EXPECT_NEAR(s.objects[0].position.z, 0.0f, 1e-3f);
}

// AN OBJECT UNDER THE CURSOR WINS EVEN IN PLACEMENT MODE. Without this, every
// attempt to select something while the palette is armed stacks a new object
// on top of it instead.
TEST(SandboxBuilderPlace, AClickOnAnExistingObjectSelectsItInsteadOfPlacing) {
    BuilderScene s;
    s.placing = true;
    s.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f, 0.0f, 0.0f), 0));
    const size_t before = s.objects.size();

    apply_builder_input(s, click_at(200.0f, 100.0f), diagonal_camera(), 400u, 200u, 0.0f);

    EXPECT_EQ(s.objects.size(), before);   // nothing was added
    EXPECT_EQ(s.selected, 0);
    EXPECT_TRUE(s.dragging);
}

TEST(SandboxBuilderPlace, PlacingOffTheGroundPlaneAddsNothing) {
    BuilderScene s;
    s.placing = true;
    // sky_camera() is pitched UP, so every pixel in the frame looks above the
    // horizon and no click can find the ground. See sky_camera's own comment
    // for why diagonal_camera() could not express this case at all.
    apply_builder_input(s, click_at(centre_of(400u), centre_of(200u)), sky_camera(), 400u, 200u,
                        0.0f);
    EXPECT_TRUE(s.objects.empty());
}

// ---------------------------------------------------------------------------
// Dragging
// ---------------------------------------------------------------------------

// A DRAG MOVES X AND Z AND MUST LEAVE Y ALONE. A drag that also changes height
// needs an axis the mouse does not have, and guessing one moves the object
// somewhere the user cannot see.
TEST(SandboxBuilderDrag, MovesInTheGroundPlaneAndLeavesHeightUntouched) {
    BuilderScene s;
    s.placing = false;
    s.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f, 0.0f, 0.0f), 0));
    const spade::render::Camera c = diagonal_camera();

    // Grab it at the centre.
    apply_builder_input(s, click_at(200.0f, 100.0f), c, 400u, 200u, 0.0f);
    ASSERT_TRUE(s.dragging);
    const float y_at_grab = s.objects[0].position.y;

    // Continue the drag to the right: down, not clicked.
    FrameInput move;
    move.mouse_x = 260.0f;
    move.mouse_y = 100.0f;
    move.left_down = true;
    apply_builder_input(s, move, c, 400u, 200u, 0.0f);

    EXPECT_GT(s.objects[0].position.x, 0.1f);            // it moved along +x
    EXPECT_NEAR(s.objects[0].position.y, y_at_grab, 1e-5f);   // and NOT in y
}

// THE GRAB OFFSET. Clicking the EDGE of an object and dragging must not snap
// its centre to the cursor -- the jump on the first frame of every drag is
// what makes a tool feel imprecise.
TEST(SandboxBuilderDrag, GrabbingOffCentreDoesNotSnapTheObjectToTheCursor) {
    BuilderScene s;
    s.placing = false;
    s.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f, 0.0f, 0.0f), 0));
    s.objects[0].scale = glm::vec3(4.0f);   // big enough to click well off centre
    s.objects[0].position.y = 2.0f;
    const spade::render::Camera c = diagonal_camera();

    const glm::vec3 before = s.objects[0].position;
    // Click off to one side but still on the object.
    apply_builder_input(s, click_at(240.0f, 110.0f), c, 400u, 200u, 0.0f);
    ASSERT_EQ(s.selected, 0);
    // The click alone must not move it at all.
    EXPECT_NEAR(s.objects[0].position.x, before.x, 1e-4f);
    EXPECT_NEAR(s.objects[0].position.z, before.z, 1e-4f);
}

// ---------------------------------------------------------------------------
// Delete and duplicate
// ---------------------------------------------------------------------------

TEST(SandboxBuilderEdit, DeleteRemovesTheSelectionAndClampsTheIndex) {
    BuilderScene s;
    for (int i = 0; i < 3; ++i) {
        s.objects.push_back(make_object_at(Shape::box, glm::vec3(float(i), 0.0f, 0.0f), i));
    }
    s.selected = 2;
    s.delete_request = true;
    apply_builder_input(s, FrameInput{}, diagonal_camera(), 400u, 200u, 0.0f);

    EXPECT_EQ(s.objects.size(), 2u);
    // CLAMPED, NOT CLEARED: deleting several in a row must not need a re-click
    // between each one.
    EXPECT_EQ(s.selected, 1);
}

TEST(SandboxBuilderEdit, DeletingTheLastObjectClearsTheSelection) {
    BuilderScene s;
    s.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f), 0));
    s.selected = 0;
    s.delete_request = true;
    apply_builder_input(s, FrameInput{}, diagonal_camera(), 400u, 200u, 0.0f);
    EXPECT_TRUE(s.objects.empty());
    EXPECT_EQ(s.selected, -1);
}

// A DUPLICATE PLACED EXACTLY ON ITS ORIGINAL LOOKS LIKE NOTHING HAPPENED, and
// the user presses the key again. The offset is the feature.
TEST(SandboxBuilderEdit, DuplicateOffsetsTheCopyAndSelectsIt) {
    BuilderScene s;
    s.objects.push_back(make_object_at(Shape::sphere, glm::vec3(0.0f), 0));
    s.selected = 0;
    s.duplicate_request = true;
    apply_builder_input(s, FrameInput{}, diagonal_camera(), 400u, 200u, 0.0f);

    ASSERT_EQ(s.objects.size(), 2u);
    EXPECT_EQ(s.selected, 1);
    EXPECT_GT(s.objects[1].position.x, s.objects[0].position.x);
    EXPECT_EQ(s.objects[1].shape, Shape::sphere);
}

// ---------------------------------------------------------------------------
// The UI-capture gate
// ---------------------------------------------------------------------------

// EVERY CLICK THAT LANDS ON A PANEL WOULD OTHERWISE ALSO PLACE AN OBJECT
// BEHIND IT. This is the single most common way a builder's UI and viewport
// fight each other.
TEST(SandboxBuilderCapture, AClickTheUiOwnsDoesNotReachTheScene) {
    BuilderScene s;
    s.placing = true;
    FrameInput in = click_at(200.0f, 100.0f);
    in.ui_captured_mouse = true;
    apply_builder_input(s, in, diagonal_camera(), 400u, 200u, 0.0f);
    EXPECT_TRUE(s.objects.empty());
}

// A DRAG THAT STARTS IN THE SCENE AND ENDS OVER A PANEL MUST STILL END.
// Suppressing the release while the UI has the mouse leaves the drag latched
// and the object keeps following the cursor after the button is up.
TEST(SandboxBuilderCapture, AReleaseOverThePanelStillEndsTheDrag) {
    BuilderScene s;
    s.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f), 0));
    s.selected = 0;
    s.dragging = true;

    FrameInput release;
    release.left_release = true;
    release.ui_captured_mouse = true;   // the pointer finished over a panel
    apply_builder_input(s, release, diagonal_camera(), 400u, 200u, 0.0f);
    EXPECT_FALSE(s.dragging);
}

// ---------------------------------------------------------------------------
// The upload contract -- the thing that silently costs the GPU path its speed
// ---------------------------------------------------------------------------

// ADDING AN OBJECT CHANGES THE MATERIAL SET AND MUST ASK FOR AN UPLOAD;
// MOVING ONE MUST NOT. If moving asked for one, the renderer would re-upload
// the world every frame of every drag and the defect would show up only as a
// frame rate -- never as a wrong picture.
TEST(SandboxBuilderUpload, PlacingNeedsAnUploadAndDraggingDoesNot) {
    BuilderScene s;
    s.placing = true;
    s.materials_dirty = false;
    const spade::render::Camera c = diagonal_camera();

    const BuilderFrameResult placed =
        apply_builder_input(s, click_at(200.0f, 100.0f), c, 400u, 200u, 0.0f);
    EXPECT_TRUE(placed.needs_upload);

    FrameInput move;
    move.mouse_x = 240.0f;
    move.mouse_y = 100.0f;
    move.left_down = true;
    const BuilderFrameResult dragged = apply_builder_input(s, move, c, 400u, 200u, 0.0f);
    EXPECT_FALSE(dragged.needs_upload);
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

TEST(SandboxBuilderMesh, BoxHasTwelveTrianglesAndUnitNormals) {
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
TEST(SandboxBuilderMesh, EveryPrimitiveIsAUnitShape) {
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
TEST(SandboxBuilderMesh, EveryTriangleFacesOutward) {
    const spade::render::MeshData meshes[3] = {make_box_mesh(), make_sphere_mesh(), make_cylinder_mesh()};
    for (int k = 0; k < 3; ++k) {
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

// ---------------------------------------------------------------------------
// Model -> RenderScene
// ---------------------------------------------------------------------------

TEST(SandboxBuilderBinding, DynamicsCarryOneDrawItemPerObjectRoutedByShape) {
    spade::render::RenderScene scene;
    scene.meshes.resize(2);       // pretend a world already loaded two meshes
    scene.materials.resize(1);    // and one material
    const BuilderBinding bind = bind_builder_meshes(scene);
    EXPECT_EQ(bind.mesh_base, 2u);
    EXPECT_EQ(bind.material_base, 1u);
    EXPECT_EQ(scene.meshes.size(), 5u);

    BuilderScene model;
    model.objects.push_back(make_object_at(Shape::cylinder, glm::vec3(0.0f), 0));
    model.objects.push_back(make_object_at(Shape::box, glm::vec3(1.0f, 0.0f, 0.0f), 1));

    sync_builder_materials(model, bind, scene);
    rebuild_builder_dynamics(model, bind, scene);

    ASSERT_EQ(scene.dynamics.size(), 2u);
    // Shape 2 (cylinder) and shape 0 (box), offset by the base -- hand-written
    // so a renumbering of the enum cannot quietly agree with itself.
    EXPECT_EQ(scene.dynamics[0].mesh_index, 4u);
    EXPECT_EQ(scene.dynamics[1].mesh_index, 2u);
    EXPECT_EQ(scene.dynamics[0].material_override, 1u);
    EXPECT_EQ(scene.dynamics[1].material_override, 2u);
    EXPECT_EQ(scene.materials.size(), 3u);   // the world's one, plus one each
}

// The base scene's own meshes and materials must survive a rebuild -- a
// builder that clobbers the ground it is standing on is an easy mistake and a
// confusing symptom.
TEST(SandboxBuilderBinding, RebuildingDoesNotDisturbTheBaseScene) {
    spade::render::RenderScene scene;
    scene.meshes.resize(1);
    scene.materials.resize(1);
    scene.materials[0].base_color = glm::vec4(0.1f, 0.2f, 0.3f, 1.0f);
    scene.statics.push_back(spade::render::DrawItem{});
    const BuilderBinding bind = bind_builder_meshes(scene);

    BuilderScene model;
    model.objects.push_back(make_object_at(Shape::box, glm::vec3(0.0f), 0));
    sync_builder_materials(model, bind, scene);
    rebuild_builder_dynamics(model, bind, scene);

    EXPECT_EQ(scene.statics.size(), 1u);
    EXPECT_NEAR(scene.materials[0].base_color.r, 0.1f, 1e-6f);
    EXPECT_NEAR(scene.materials[0].base_color.b, 0.3f, 1e-6f);
}
