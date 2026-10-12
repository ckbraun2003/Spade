// The transform gizmo: move along an axis or in a plane, or turn about an
// axis, with snapping (Unity's and Unreal's basic; the lead's ask after editor
// Task 12). Display-free: the session gives it the camera and the view's rays,
// applies the poses it returns through EditorWorkspace::drag() (one undo per
// drag), and draws the lines gizmo_lines() returns.
//
// The gizmo is world-aligned and keeps one size on screen. Its handles:
//   move:   three arms (an axis each) and three small squares (a plane each,
//           named by its normal), from the object's centre;
//   rotate: three rings (an axis each) of the arms' length.
// A handle seen edge-on is neither drawn nor hit.

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/target.hpp"

namespace spade::sandbox::editor {

enum class GizmoTool { move, rotate };

struct GizmoHandle {
    enum class Kind { none, axis, plane, ring };
    Kind kind = Kind::none;
    int axis = 0;  // 0 x, 1 y, 2 z: an arm's or a ring's axis, a plane's normal

    friend bool operator==(const GizmoHandle&, const GizmoHandle&) = default;
};

// Snapping: a move puts the moved axes on a grid of `move` metres; a turn is
// rounded to `rotate_deg` degrees.
struct Snap {
    bool on = false;
    float move = 0.25f;
    float rotate_deg = 15.0f;
};

// What a drag changes: an asset's pose, or a vehicle's start, without its scale.
struct GizmoPose {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// A segment the view draws, in the view's pixels (+y down), with its axis for
// its colour and whether it belongs to the handle under the cursor or dragged.
struct GizmoLine {
    glm::vec2 a{0.0f};
    glm::vec2 b{0.0f};
    int axis = 0;
    bool hot = false;
};

inline constexpr float kGizmoViewFraction = 0.12f;  // an arm's length on screen, of the view's height
inline constexpr float kHandleTolerance = 0.08f;    // how near a ray must pass, of an arm's length
inline constexpr float kEdgeOn = 0.1f;              // |cos| between the view and a plane below which it is edge-on

namespace detail {

[[nodiscard]] inline glm::vec3 unit(int axis) noexcept {
    glm::vec3 e(0.0f);
    e[axis] = 1.0f;
    return e;
}

// The ring's in-plane basis, u x v = the axis, so a turn of +a moves u toward v.
[[nodiscard]] inline std::array<glm::vec3, 2> ring_basis(int axis) noexcept {
    return {unit((axis + 1) % 3), unit((axis + 2) % 3)};
}

[[nodiscard]] inline glm::vec3 forward(const render::Camera& cam) noexcept {
    return glm::normalize(cam.orientation * glm::vec3(0.0f, 0.0f, -1.0f));
}

// Where the ray meets the plane through `point` with normal `n`; none when
// the ray runs along the plane or the meeting is behind the ray's origin.
[[nodiscard]] inline std::optional<float> ray_plane(glm::vec3 o, glm::vec3 d, glm::vec3 point, glm::vec3 n) noexcept {
    const float den = glm::dot(d, n);
    if (std::abs(den) < 1e-6f) return std::nullopt;
    const float t = glm::dot(point - o, n) / den;
    if (t < 0.0f) return std::nullopt;
    return t;
}

// The parameters of the closest points of the ray o + t d (t >= 0) and the
// line c + s e; none when they are parallel.
struct Closest {
    float t = 0.0f;
    float s = 0.0f;
    float distance = 0.0f;
};
[[nodiscard]] inline std::optional<Closest> ray_line(glm::vec3 o, glm::vec3 d, glm::vec3 c, glm::vec3 e) noexcept {
    const glm::vec3 w = o - c;
    const float b = glm::dot(d, e);
    const float den = 1.0f - b * b;  // d and e are unit
    if (den < 1e-8f) return std::nullopt;
    const float dw = glm::dot(d, w);
    const float ew = glm::dot(e, w);
    Closest r;
    r.t = std::max(0.0f, (b * ew - dw) / den);
    r.s = glm::dot(o + r.t * d - c, e);
    r.distance = glm::length(o + r.t * d - (c + r.s * e));
    return r;
}

[[nodiscard]] inline float snapped(float v, float step) noexcept {
    return step > 0.0f ? std::round(v / step) * step : v;
}

[[nodiscard]] inline bool edge_on(const render::Camera& cam, glm::vec3 centre, int normal_axis) noexcept {
    const glm::vec3 view = glm::normalize(centre - cam.position);
    return std::abs(glm::dot(view, unit(normal_axis))) < kEdgeOn;
}

}  // namespace detail

// Where `p` is drawn in a view of `width` x `height` pixels (+y down), the
// inverse of ray_from_screen(); none behind the camera.
[[nodiscard]] inline std::optional<glm::vec2> project(const render::Camera& cam, glm::vec3 p, uint32_t width,
                                                      uint32_t height) noexcept {
    const float w = static_cast<float>(width == 0u ? 1u : width);
    const float h = static_cast<float>(height == 0u ? 1u : height);
    const glm::vec3 c = glm::conjugate(cam.orientation) * (p - cam.position);
    if (c.z >= -1e-6f) return std::nullopt;
    const float tan_half = std::tan(cam.fov_y_radians * 0.5f);
    const float ndc_x = c.x / -c.z / (w / h * tan_half);
    const float ndc_y = c.y / -c.z / tan_half;
    return glm::vec2((ndc_x + 1.0f) * 0.5f * w - 0.5f, (1.0f - ndc_y) * 0.5f * h - 0.5f);
}

// An arm's length in metres, so that the gizmo keeps one size on screen.
[[nodiscard]] inline float gizmo_size(const render::Camera& cam, glm::vec3 centre) noexcept {
    const float depth = std::max(glm::dot(centre - cam.position, detail::forward(cam)), cam.near_plane);
    return depth * 2.0f * std::tan(cam.fov_y_radians * 0.5f) * kGizmoViewFraction;
}

// The handle the ray from `origin` along `direction` (unit) passes nearest the
// camera, among the tool's; none when it passes none.
[[nodiscard]] inline GizmoHandle gizmo_hit(GizmoTool tool, const render::Camera& cam, glm::vec3 centre,
                                           glm::vec3 origin, glm::vec3 direction) noexcept {
    const float l = gizmo_size(cam, centre);
    const float tol = kHandleTolerance * l;
    GizmoHandle best;
    float best_t = INFINITY;
    const auto offer = [&](GizmoHandle h, float t) {
        if (t < best_t) {
            best_t = t;
            best = h;
        }
    };
    for (int a = 0; a < 3; ++a) {
        const glm::vec3 e = detail::unit(a);
        if (tool == GizmoTool::move) {
            if (const std::optional<detail::Closest> c = detail::ray_line(origin, direction, centre, e)) {
                if (c->distance < tol && c->s > 0.15f * l && c->s < 1.1f * l) offer({GizmoHandle::Kind::axis, a}, c->t);
            }
            if (!detail::edge_on(cam, centre, a)) {
                if (const std::optional<float> t = detail::ray_plane(origin, direction, centre, e)) {
                    const glm::vec3 q = origin + *t * direction - centre;
                    const float u = q[(a + 1) % 3];
                    const float v = q[(a + 2) % 3];
                    if (u > 0.2f * l && u < 0.4f * l && v > 0.2f * l && v < 0.4f * l) {
                        offer({GizmoHandle::Kind::plane, a}, *t);
                    }
                }
            }
        } else if (!detail::edge_on(cam, centre, a)) {
            if (const std::optional<float> t = detail::ray_plane(origin, direction, centre, e)) {
                const float r = glm::length(origin + *t * direction - centre);
                if (std::abs(r - l) < tol) offer({GizmoHandle::Kind::ring, a}, *t);
            }
        }
    }
    return best;
}

// One drag of one handle, from the pose the object had when it was pressed.
class GizmoDrag {
  public:
    // Starts a drag of `handle` by the ray that pressed it; none when that ray
    // cannot hold the handle (it runs along the axis, or misses the plane).
    [[nodiscard]] static std::optional<GizmoDrag> begin(GizmoHandle handle, GizmoPose start, glm::vec3 origin,
                                                        glm::vec3 direction) noexcept {
        GizmoDrag d;
        d.handle_ = handle;
        d.start_ = start;
        const std::optional<float> at = d.measure(origin, direction);
        if (!at && handle.kind != GizmoHandle::Kind::plane) return std::nullopt;
        if (handle.kind == GizmoHandle::Kind::plane) {
            const std::optional<glm::vec3> p = d.on_plane(origin, direction);
            if (!p) return std::nullopt;
            d.from_point_ = *p;
        } else {
            d.from_ = *at;
        }
        return d;
    }

    // The pose the ray now asks for. A ray that cannot hold the handle keeps
    // the pose the drag started from.
    [[nodiscard]] GizmoPose update(glm::vec3 origin, glm::vec3 direction, const Snap& snap) const noexcept {
        GizmoPose pose = start_;
        const int a = handle_.axis;
        switch (handle_.kind) {
            case GizmoHandle::Kind::axis: {
                const std::optional<float> s = measure(origin, direction);
                if (!s) return pose;
                pose.position[a] += *s - from_;
                if (snap.on) pose.position[a] = detail::snapped(pose.position[a], snap.move);
                return pose;
            }
            case GizmoHandle::Kind::plane: {
                const std::optional<glm::vec3> p = on_plane(origin, direction);
                if (!p) return pose;
                for (const int b : {(a + 1) % 3, (a + 2) % 3}) {
                    pose.position[b] += (*p)[b] - from_point_[b];
                    if (snap.on) pose.position[b] = detail::snapped(pose.position[b], snap.move);
                }
                return pose;
            }
            case GizmoHandle::Kind::ring: {
                const std::optional<float> angle = measure(origin, direction);
                if (!angle) return pose;
                float turn = *angle - from_;
                constexpr float kPi = std::numbers::pi_v<float>;
                if (turn > kPi) turn -= 2.0f * kPi;
                if (turn < -kPi) turn += 2.0f * kPi;
                if (snap.on) turn = detail::snapped(turn, snap.rotate_deg * kPi / 180.0f);
                pose.rotation = glm::normalize(glm::angleAxis(turn, detail::unit(a)) * start_.rotation);
                return pose;
            }
            case GizmoHandle::Kind::none: break;
        }
        return pose;
    }

    [[nodiscard]] GizmoHandle handle() const noexcept { return handle_; }
    [[nodiscard]] const GizmoPose& start() const noexcept { return start_; }

  private:
    GizmoDrag() = default;

    // An axis's: how far along it the ray passes nearest. A ring's: the
    // ray's angle about it, where it meets the ring's plane.
    [[nodiscard]] std::optional<float> measure(glm::vec3 o, glm::vec3 d) const noexcept {
        const int a = handle_.axis;
        if (handle_.kind == GizmoHandle::Kind::axis) {
            const std::optional<detail::Closest> c = detail::ray_line(o, d, start_.position, detail::unit(a));
            if (!c) return std::nullopt;
            return c->s;
        }
        if (handle_.kind == GizmoHandle::Kind::ring) {
            const std::optional<glm::vec3> p = on_plane(o, d);
            if (!p) return std::nullopt;
            const std::array<glm::vec3, 2> uv = detail::ring_basis(a);
            const glm::vec3 r = *p - start_.position;
            return std::atan2(glm::dot(r, uv[1]), glm::dot(r, uv[0]));
        }
        return std::nullopt;
    }

    // Where the ray meets the handle's plane through the start position.
    [[nodiscard]] std::optional<glm::vec3> on_plane(glm::vec3 o, glm::vec3 d) const noexcept {
        const std::optional<float> t = detail::ray_plane(o, d, start_.position, detail::unit(handle_.axis));
        if (!t) return std::nullopt;
        return o + *t * d;
    }

    GizmoHandle handle_;
    GizmoPose start_;
    float from_ = 0.0f;
    glm::vec3 from_point_{0.0f};
};

// The lines the view draws for the gizmo at `centre`, `hot` marked. A segment
// with an end off the view or behind the camera is left out.
[[nodiscard]] inline std::vector<GizmoLine> gizmo_lines(GizmoTool tool, const render::Camera& cam, uint32_t width,
                                                        uint32_t height, glm::vec3 centre, GizmoHandle hot) {
    std::vector<GizmoLine> out;
    const float l = gizmo_size(cam, centre);
    const auto inside = [&](glm::vec2 p) {
        return p.x >= 0.0f && p.y >= 0.0f && p.x <= static_cast<float>(width) && p.y <= static_cast<float>(height);
    };
    const auto add2 = [&](glm::vec2 a, glm::vec2 b, int axis, bool is_hot) {
        if (inside(a) && inside(b)) out.push_back(GizmoLine{a, b, axis, is_hot});
    };
    const auto add = [&](glm::vec3 a, glm::vec3 b, int axis, bool is_hot) {
        const std::optional<glm::vec2> pa = project(cam, a, width, height);
        const std::optional<glm::vec2> pb = project(cam, b, width, height);
        if (pa && pb) add2(*pa, *pb, axis, is_hot);
    };
    for (int a = 0; a < 3; ++a) {
        const glm::vec3 e = detail::unit(a);
        if (tool == GizmoTool::move) {
            const bool arm_hot = hot == GizmoHandle{GizmoHandle::Kind::axis, a};
            add(centre + 0.15f * l * e, centre + l * e, a, arm_hot);
            // The arrowhead, drawn on screen across the arm's end.
            const std::optional<glm::vec2> tip = project(cam, centre + l * e, width, height);
            const std::optional<glm::vec2> back = project(cam, centre + 0.85f * l * e, width, height);
            if (tip && back && glm::length(*tip - *back) > 1e-3f) {
                const glm::vec2 along = glm::normalize(*tip - *back);
                const glm::vec2 across(-along.y, along.x);
                const float size = glm::length(*tip - *back);
                add2(*tip, *tip - along * size + across * size * 0.5f, a, arm_hot);
                add2(*tip, *tip - along * size - across * size * 0.5f, a, arm_hot);
            }
            if (!detail::edge_on(cam, centre, a)) {
                const bool plane_hot = hot == GizmoHandle{GizmoHandle::Kind::plane, a};
                const glm::vec3 u = 0.2f * l * detail::unit((a + 1) % 3);
                const glm::vec3 v = 0.2f * l * detail::unit((a + 2) % 3);
                const glm::vec3 c0 = centre + u + v;  // the square from 0.2 to 0.4 of an arm
                add(c0, c0 + u, a, plane_hot);
                add(c0 + u, c0 + u + v, a, plane_hot);
                add(c0 + u + v, c0 + v, a, plane_hot);
                add(c0 + v, c0, a, plane_hot);
            }
        } else if (!detail::edge_on(cam, centre, a)) {
            const bool ring_hot = hot == GizmoHandle{GizmoHandle::Kind::ring, a};
            const std::array<glm::vec3, 2> uv = detail::ring_basis(a);
            constexpr int kSegments = 48;
            for (int i = 0; i < kSegments; ++i) {
                const float p0 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(kSegments);
                const float p1 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i + 1) / static_cast<float>(kSegments);
                add(centre + l * (uv[0] * std::cos(p0) + uv[1] * std::sin(p0)),
                    centre + l * (uv[0] * std::cos(p1) + uv[1] * std::sin(p1)), a, ring_hot);
            }
        }
    }
    return out;
}

}  // namespace spade::sandbox::editor
