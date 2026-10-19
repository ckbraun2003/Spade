// The transform gizmo (the lead's ask after editor Task 12; Unity's and
// Unreal's basic): where it is drawn, what a ray hits, and what a drag along
// an axis, in a plane or about an axis makes of the pose, with snapping.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/screen_ray.hpp"  // ray_from_screen()
#include "../sandbox/editor_gizmo.hpp"
#include "../sandbox/orbit_camera.hpp"

namespace {

namespace ed = spade::sandbox::editor;
using ed::GizmoHandle;
using ed::GizmoTool;
using Kind = ed::GizmoHandle::Kind;

constexpr uint32_t kW = 800;
constexpr uint32_t kH = 600;
const glm::vec3 kCentre(1.0f, 0.5f, -0.5f);

// The editor's camera, orbiting the centre from 5 m, turned so that no handle
// is seen edge-on (an edge-on plane or ring is not drawn, as in Unity).
[[nodiscard]] spade::render::Camera camera() {
    spade::sandbox::OrbitCamera c;
    c.target = kCentre;
    c.distance = 5.0f;
    c.yaw = 0.6f;
    return c.to_render_camera();
}

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

// The view's ray through the pixel where `p` is drawn: what a click there gives.
[[nodiscard]] Ray ray_at(const spade::render::Camera& cam, glm::vec3 p) {
    const std::optional<glm::vec2> px = ed::project(cam, p, kW, kH);
    EXPECT_TRUE(px.has_value());
    const glm::vec2 at = px.value_or(glm::vec2(0.0f));
    const spade::sandbox::Ray r = spade::sandbox::ray_from_screen(cam, at.x, at.y, kW, kH);
    return Ray{r.origin, r.direction};
}

[[nodiscard]] glm::vec3 unit(int axis) {
    glm::vec3 e(0.0f);
    e[axis] = 1.0f;
    return e;
}

[[nodiscard]] GizmoHandle hit_at(GizmoTool tool, glm::vec3 p) {
    const spade::render::Camera cam = camera();
    const Ray r = ray_at(cam, p);
    return ed::gizmo_hit(tool, cam, kCentre, r.origin, r.direction);
}

[[nodiscard]] ed::GizmoPose start_pose() {
    return ed::GizmoPose{kCentre, glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 2.0f, 0.5f)))};
}

// Equal rotations, either sign of the quaternion.
[[nodiscard]] bool same_rotation(glm::quat a, glm::quat b) { return std::abs(glm::dot(a, b)) > 1.0f - 1e-5f; }

}  // namespace

TEST(EditorGizmo, TheProjectionInvertsTheViewsRay) {
    const spade::render::Camera cam = camera();
    for (const glm::vec2 px : {glm::vec2(10.0f, 20.0f), glm::vec2(400.0f, 300.0f), glm::vec2(790.0f, 590.0f)}) {
        const spade::sandbox::Ray r = spade::sandbox::ray_from_screen(cam, px.x, px.y, kW, kH);
        const std::optional<glm::vec2> back = ed::project(cam, r.origin + 3.0f * r.direction, kW, kH);
        ASSERT_TRUE(back.has_value());
        EXPECT_NEAR(back->x, px.x, 1e-2f);
        EXPECT_NEAR(back->y, px.y, 1e-2f);
    }
    // Behind the camera is not drawn.
    const spade::sandbox::Ray r = spade::sandbox::ray_from_screen(cam, 400.0f, 300.0f, kW, kH);
    EXPECT_FALSE(ed::project(cam, r.origin - r.direction, kW, kH).has_value());
}

// The gizmo keeps its size on screen: its arms grow with its distance.
TEST(EditorGizmo, ItsSizeFollowsItsDistance) {
    spade::sandbox::OrbitCamera c;
    c.target = kCentre;
    c.distance = 5.0f;
    const float near = ed::gizmo_size(c.to_render_camera(), kCentre);
    c.distance = 10.0f;
    const float far = ed::gizmo_size(c.to_render_camera(), kCentre);
    EXPECT_GT(near, 0.0f);
    EXPECT_NEAR(far / near, 2.0f, 1e-3f);
}

TEST(EditorGizmo, EachMoveHandleIsHitWhereItIsDrawn) {
    const float l = ed::gizmo_size(camera(), kCentre);
    for (int a = 0; a < 3; ++a) {
        EXPECT_EQ(hit_at(GizmoTool::move, kCentre + 0.7f * l * unit(a)), (GizmoHandle{Kind::axis, a})) << a;
        const glm::vec3 in_plane = unit((a + 1) % 3) + unit((a + 2) % 3);
        EXPECT_EQ(hit_at(GizmoTool::move, kCentre + 0.3f * l * in_plane), (GizmoHandle{Kind::plane, a})) << a;
    }
    EXPECT_EQ(hit_at(GizmoTool::move, kCentre + 3.0f * l * glm::vec3(1.0f, 1.0f, 0.0f)), GizmoHandle{});
}

TEST(EditorGizmo, EachRingIsHitOnItsCircle) {
    const float l = ed::gizmo_size(camera(), kCentre);
    for (int a = 0; a < 3; ++a) {
        const glm::vec3 on_ring = l * glm::normalize(unit((a + 1) % 3) + unit((a + 2) % 3));
        EXPECT_EQ(hit_at(GizmoTool::rotate, kCentre + on_ring), (GizmoHandle{Kind::ring, a})) << a;
    }
    EXPECT_EQ(hit_at(GizmoTool::rotate, kCentre + 3.0f * l * glm::vec3(1.0f, 1.0f, 0.0f)), GizmoHandle{});
}

TEST(EditorGizmo, AnAxisDragMovesAlongItsAxisOnly) {
    const spade::render::Camera cam = camera();
    const float l = ed::gizmo_size(cam, kCentre);
    const Ray press = ray_at(cam, kCentre + 0.6f * l * unit(0));
    const std::optional<ed::GizmoDrag> drag =
        ed::GizmoDrag::begin(GizmoHandle{Kind::axis, 0}, start_pose(), press.origin, press.direction);
    ASSERT_TRUE(drag.has_value());
    const Ray now = ray_at(cam, kCentre + (0.6f * l + 0.8f) * unit(0));
    const ed::GizmoPose moved = drag->update(now.origin, now.direction, ed::Snap{});
    EXPECT_NEAR(moved.position.x, kCentre.x + 0.8f, 1e-3f);
    EXPECT_EQ(moved.position.y, kCentre.y);
    EXPECT_EQ(moved.position.z, kCentre.z);
    EXPECT_TRUE(same_rotation(moved.rotation, start_pose().rotation));
}

TEST(EditorGizmo, APlaneDragStaysInItsPlane) {
    const spade::render::Camera cam = camera();
    const float l = ed::gizmo_size(cam, kCentre);
    const glm::vec3 handle = kCentre + 0.3f * l * glm::vec3(1.0f, 0.0f, 1.0f);  // the ground plane's
    const Ray press = ray_at(cam, handle);
    const std::optional<ed::GizmoDrag> drag =
        ed::GizmoDrag::begin(GizmoHandle{Kind::plane, 1}, start_pose(), press.origin, press.direction);
    ASSERT_TRUE(drag.has_value());
    const Ray now = ray_at(cam, handle + glm::vec3(0.5f, 0.0f, -0.7f));
    const ed::GizmoPose moved = drag->update(now.origin, now.direction, ed::Snap{});
    EXPECT_EQ(moved.position.y, kCentre.y);
    EXPECT_NEAR(moved.position.x, kCentre.x + 0.5f, 1e-3f);
    EXPECT_NEAR(moved.position.z, kCentre.z - 0.7f, 1e-3f);
}

// Snapping puts the moved axes on the grid; the others keep their values.
TEST(EditorGizmo, SnappingPutsTheMovedAxisOnTheGrid) {
    const spade::render::Camera cam = camera();
    const float l = ed::gizmo_size(cam, kCentre);
    const Ray press = ray_at(cam, kCentre + 0.6f * l * unit(0));
    const std::optional<ed::GizmoDrag> drag =
        ed::GizmoDrag::begin(GizmoHandle{Kind::axis, 0}, start_pose(), press.origin, press.direction);
    ASSERT_TRUE(drag.has_value());
    const Ray now = ray_at(cam, kCentre + (0.6f * l + 0.8f) * unit(0));
    ed::Snap snap;
    snap.on = true;
    snap.move = 0.25f;
    const ed::GizmoPose moved = drag->update(now.origin, now.direction, snap);
    EXPECT_EQ(moved.position.x, 1.75f);  // 1.0 + 0.8, to the nearest 0.25
    EXPECT_EQ(moved.position.y, kCentre.y);
    EXPECT_EQ(moved.position.z, kCentre.z);
}

TEST(EditorGizmo, ARingDragTurnsAboutItsAxisAndSnapsTheTurn) {
    const spade::render::Camera cam = camera();
    const float l = ed::gizmo_size(cam, kCentre);
    // Around +Y, in a basis with u x v = +Y.
    const glm::vec3 u(0.0f, 0.0f, 1.0f);
    const glm::vec3 v(1.0f, 0.0f, 0.0f);
    const auto on_ring = [&](float phi) { return kCentre + l * (u * std::cos(phi) + v * std::sin(phi)); };
    const Ray press = ray_at(cam, on_ring(0.6f));
    const std::optional<ed::GizmoDrag> drag =
        ed::GizmoDrag::begin(GizmoHandle{Kind::ring, 1}, start_pose(), press.origin, press.direction);
    ASSERT_TRUE(drag.has_value());
    const Ray now = ray_at(cam, on_ring(1.1f));
    const ed::GizmoPose turned = drag->update(now.origin, now.direction, ed::Snap{});
    EXPECT_TRUE(same_rotation(turned.rotation, glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f)) * start_pose().rotation));
    EXPECT_EQ(turned.position, kCentre);

    ed::Snap snap;
    snap.on = true;
    snap.rotate_deg = 15.0f;
    const ed::GizmoPose snapped = drag->update(now.origin, now.direction, snap);  // 28.6 deg -> 30
    EXPECT_TRUE(same_rotation(snapped.rotation,
                              glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f)) * start_pose().rotation));
}

// The lines the view draws: inside it, and the hot handle marked.
TEST(EditorGizmo, ItsLinesAreInTheViewAndMarkTheHotHandle) {
    for (const GizmoTool tool : {GizmoTool::move, GizmoTool::rotate}) {
        const GizmoHandle hot = tool == GizmoTool::move ? GizmoHandle{Kind::axis, 2} : GizmoHandle{Kind::ring, 2};
        const std::vector<ed::GizmoLine> lines = ed::gizmo_lines(tool, camera(), kW, kH, kCentre, hot);
        ASSERT_FALSE(lines.empty());
        bool marked = false;
        for (const ed::GizmoLine& g : lines) {
            for (const glm::vec2 p : {g.a, g.b}) {
                EXPECT_TRUE(p.x >= 0.0f && p.x <= static_cast<float>(kW) && p.y >= 0.0f && p.y <= static_cast<float>(kH));
            }
            if (g.hot) {
                EXPECT_EQ(g.axis, 2);
                marked = true;
            }
        }
        EXPECT_TRUE(marked);
    }
}
