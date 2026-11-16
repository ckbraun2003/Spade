// A model's contact shape in the editor (L82, the design note docs/design/interface/plans/
// 2026-11-16-l82-contact-shapes-in-the-editor-draft.md and its rulings, L82a). Display-free (SL15b).
//
// The shape is the engine's (physics::ShapeDesc, L68), and it lives on the MODEL (ModelType::shape): the editor decides
// nothing about it that the engine does not, except the three things below.
//   check_shape        what the inspector asks before an edit: the engine's own refusals by name, and Q7, a box
//                      refused when the model's design frame is not axis-aligned (a turned frame permutes a box's
//                      half extents, and the first cut says so instead of doing it).
//   hull_from_model    the "From model" button (Q6): the hull of the body parts' eight corners and the rotor tips, in the
//                      BODY frame the engine keeps shapes in, refused by name when it needs more than the engine's 16.
//   box_inertia_diag   the "Inertia from box" button (Q3): the solid box's principal moments, for the model's mass.
// And the wireframe: shape_segments() draws a shape in its own (body) frame, body_pose() is where that frame is for a
// vehicle at a design pose (the engine's one conversion, sim/design_frame.hpp), and the caller places the segments.
//
// FRAMES. A shape's points are in the principal frame at the centre of mass (rigid_contact.hpp); the inspector's design
// is in the model's design frame. A design point p_d is the body point q_bd * (p_d - com_offset).

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "editor_gizmo.hpp"  // GizmoLine and project(): the wireframe is drawn through the gizmo's line path
#include "physics/rigid_contact.hpp"
#include "render/target.hpp"
#include "sim/design_frame.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/model_type.hpp"

namespace spade::sandbox::editor {

// One line of a wireframe, in whatever frame the caller holds the shape in.
struct Segment {
    glm::vec3 a{0.0f};
    glm::vec3 b{0.0f};
};

namespace detail {

[[nodiscard]] inline std::string quoted(const vehicles::ModelType& m) { return "model '" + m.name + "': "; }

// A rotation that turns each axis onto an axis: every rotated basis vector has one component of magnitude 1.
[[nodiscard]] inline bool axis_aligned(const glm::quat& q) {
    for (const glm::vec3 axis : {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)}) {
        const glm::vec3 r = q * axis;
        const float biggest = std::max({std::fabs(r.x), std::fabs(r.y), std::fabs(r.z)});
        if (biggest < 1.0f - 1e-4f) return false;
    }
    return true;
}

}  // namespace detail

// The engine's refusals (compile_shape: every text opens "set_body_shape:"), then Q7. The model's name leads, so the
// inspector's status line says which model.
[[nodiscard]] inline Result<void> check_shape(const vehicles::ModelType& model, const physics::ShapeDesc& shape) {
    if (const Result<physics::ShapeRow> row = physics::compile_shape(shape); !row) {
        return std::unexpected(Error{row.error().code, detail::quoted(model) + row.error().context});
    }
    if (shape.kind == physics::ShapeKind::box &&
        !detail::axis_aligned(vehicles::canonical_design_rotation(model.design_to_principal))) {
        return std::unexpected(Error{Code::invalid_argument,
                                     detail::quoted(model) +
                                         "a box needs a design frame aligned with the principal axes, and this model's "
                                         "design_to_principal turns them by something other than a multiple of 90 "
                                         "degrees (a quarter turn only permutes the axes); use a hull, or set the "
                                         "design frame straight"});
    }
    return {};
}

// Bit for bit (so -0 and 0 differ and a NaN equals itself): "no change" must mean the saved file would not change, as
// same_value() is for a model parameter.
[[nodiscard]] inline bool same_shape(const std::optional<physics::ShapeDesc>& a, const std::optional<physics::ShapeDesc>& b) {
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    const auto bits = [](float f) { return std::bit_cast<uint32_t>(f); };
    const auto same3 = [&](const glm::vec3& x, const glm::vec3& y) {
        return bits(x.x) == bits(y.x) && bits(x.y) == bits(y.y) && bits(x.z) == bits(y.z);
    };
    if (a->kind != b->kind || bits(a->radius) != bits(b->radius) || !same3(a->a, b->a) || !same3(a->b, b->b) ||
        !same3(a->half_extents, b->half_extents) || a->points.size() != b->points.size() ||
        bits(a->friction) != bits(b->friction) || bits(a->restitution) != bits(b->restitution) ||
        bits(a->rest_speed) != bits(b->rest_speed) || a->iterations != b->iterations) {
        return false;
    }
    for (std::size_t i = 0; i < a->points.size(); ++i) {
        if (!same3(a->points[i], b->points[i])) return false;
    }
    return true;
}

// 1 / box_inv_inertia: the principal moments of a solid box of `mass`, kg m^2, about its centre.
[[nodiscard]] inline glm::vec3 box_inertia_diag(float mass, glm::vec3 half_extents) {
    const float x2 = half_extents.x * half_extents.x;
    const float y2 = half_extents.y * half_extents.y;
    const float z2 = half_extents.z * half_extents.z;
    return glm::vec3(mass * (y2 + z2) / 3.0f, mass * (x2 + z2) / 3.0f, mass * (x2 + y2) / 3.0f);
}

// The "Inertia from box" button's value (Q3): the model's body.inertia_diag as a solid box of the model's mass with the shape's
// half extents. Only when the model's shape is a box; nothing is set automatically.
[[nodiscard]] inline std::optional<glm::vec3> inertia_from_box(const vehicles::ModelType& model) {
    if (!model.shape || model.shape->kind != physics::ShapeKind::box) return std::nullopt;
    return box_inertia_diag(model.body.mass, model.shape->half_extents);
}

// The body parts' union box in the DESIGN frame, (lo, hi): each part's box (vehicles::part_bounds(), in the part's frame)
// posed by its mount. A rotor's parts are left out (they turn; their reach is the rotor's). Refused by name with no parts, or
// when no body part has a box.
[[nodiscard]] inline Result<std::pair<glm::vec3, glm::vec3>> parts_box(const vehicles::ModelType& model) {
    if (model.parts.empty()) {
        return std::unexpected(Error{Code::invalid_argument, detail::quoted(model) + "it has no parts to size a shape from"});
    }
    glm::vec3 lo(1e30f);
    glm::vec3 hi(-1e30f);
    bool any = false;
    std::string why;
    for (const vehicles::PartDesc& part : model.parts) {
        if (part.attach != vehicles::PartAttach::body) continue;
        const Result<Aabb> box = vehicles::part_bounds(part);
        if (!box) {
            why = box.error().context;
            continue;
        }
        for (int c = 0; c < 8; ++c) {
            const glm::vec3 corner((c & 1) ? box->max.x : box->min.x, (c & 2) ? box->max.y : box->min.y,
                                   (c & 4) ? box->max.z : box->min.z);
            const glm::vec3 p = part.mount_pos + part.mount_orient * corner;
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        any = true;
    }
    if (!any) {
        return std::unexpected(Error{Code::invalid_argument,
                                     detail::quoted(model) + "none of its body parts has a box to size a shape from" +
                                         (why.empty() ? std::string{} : " (" + why + ")")});
    }
    return std::pair{lo, hi};
}

// The hull "From model" builds. The body parts' boxes (vehicles::part_bounds(), posed by their mounts) are united into one
// box in the design frame and its eight corners are the first points; each rotor adds a tip, its hub pushed out along the
// line from the centre of mass by the rotor's radius (the rule model_reach() uses for a tip). All in the body frame.
[[nodiscard]] inline Result<physics::ShapeDesc> hull_from_model(const vehicles::ModelType& model) {
    const Result<std::pair<glm::vec3, glm::vec3>> box = parts_box(model);
    if (!box) return std::unexpected(box.error());
    const auto [lo, hi] = *box;
    const std::size_t needed = 8u + model.rotors.size();
    if (needed > physics::kShapeMaxPoints) {
        return std::unexpected(Error{Code::invalid_argument,
                                     detail::quoted(model) + "its hull would need " + std::to_string(needed) +
                                         " points (8 corners and " + std::to_string(model.rotors.size()) +
                                         " rotor tips) and a hull holds " + std::to_string(physics::kShapeMaxPoints) +
                                         "; use a box, or a model with fewer rotors"});
    }
    const glm::quat q_bd = vehicles::canonical_design_rotation(model.design_to_principal);
    std::vector<glm::vec3> design;
    for (int c = 0; c < 8; ++c) {
        design.emplace_back((c & 1) ? hi.x : lo.x, (c & 2) ? hi.y : lo.y, (c & 4) ? hi.z : lo.z);
    }
    for (const vehicles::RotorDesc& rotor : model.rotors) {
        const glm::vec3 out = rotor.local_pos - model.com_offset;
        const float length = glm::length(out);
        const glm::vec3 direction = length > 1e-6f ? out / length : glm::vec3(1.0f, 0.0f, 0.0f);
        design.push_back(rotor.local_pos + direction * rotor.radius);
    }
    std::vector<glm::vec3> body;
    body.reserve(design.size());
    for (const glm::vec3& p : design) body.push_back(q_bd * (p - model.com_offset));
    return physics::hull_shape(std::move(body));
}

// Where the shape's frame is for a vehicle whose DESIGN origin is at `pos` with `orient`: the body's position (its centre of
// mass) and orientation, by the engine's own conversion (the spawn's and vehicle_state()'s).
[[nodiscard]] inline FrameState body_pose(const vehicles::ModelType& model, glm::vec3 pos, glm::quat orient) {
    FrameState design;
    design.pos = pos;
    design.orient = orient;
    return to_body_state(design, vehicles::canonical_design_rotation(model.design_to_principal), model.com_offset);
}

// The half extents, in the BODY frame, of the box about the centre of mass that holds the body parts' box: each axis the
// largest |coordinate| of the box's eight corners seen from the centre of mass. Refused as parts_box() is.
[[nodiscard]] inline Result<glm::vec3> body_half_extents(const vehicles::ModelType& model) {
    const Result<std::pair<glm::vec3, glm::vec3>> box = parts_box(model);
    if (!box) return std::unexpected(box.error());
    const glm::quat q_bd = vehicles::canonical_design_rotation(model.design_to_principal);
    glm::vec3 half(0.0f);
    for (int c = 0; c < 8; ++c) {
        const glm::vec3 corner((c & 1) ? box->second.x : box->first.x, (c & 2) ? box->second.y : box->first.y,
                               (c & 4) ? box->second.z : box->first.z);
        half = glm::max(half, glm::abs(q_bd * (corner - model.com_offset)));
    }
    return half;
}

// What the inspector gives when a kind is picked from its list: sizes from the body parts' box (body_half_extents), or plain
// ones (sphere 0.1 m, box 0.1 m, a capsule of radius 0.05 m along x, 0.2 m long) for a model whose parts give none. A box takes
// the half extents; a sphere the largest; a capsule lies along the longest axis (x before y before z on a tie) with the smaller
// of the other two as its radius, its ends at +-(h - r), no nearer the middle than r/2. The hull is hull_from_model(), and a
// model with no parts has none. `none` is the empty description (the inspector clears the shape instead).
[[nodiscard]] inline Result<physics::ShapeDesc> shape_for_kind(const vehicles::ModelType& model, physics::ShapeKind kind) {
    using physics::ShapeKind;
    if (kind == ShapeKind::none) return physics::ShapeDesc{};
    if (kind == ShapeKind::hull) return hull_from_model(model);
    const Result<glm::vec3> sized = body_half_extents(model);
    if (kind == ShapeKind::sphere) return physics::sphere_shape(sized ? std::max({sized->x, sized->y, sized->z}) : 0.1f);
    if (kind == ShapeKind::box) return physics::box_shape(sized ? *sized : glm::vec3(0.1f));
    if (!sized) return physics::capsule_shape(glm::vec3(-0.1f, 0.0f, 0.0f), glm::vec3(0.1f, 0.0f, 0.0f), 0.05f);
    int axis = 0;
    for (int i = 1; i < 3; ++i) {
        if ((*sized)[i] > (*sized)[axis]) axis = i;
    }
    float radius = 1e30f;
    for (int i = 0; i < 3; ++i) {
        if (i != axis) radius = std::min(radius, (*sized)[i]);
    }
    const float reach = std::max((*sized)[axis] - radius, 0.5f * radius);
    glm::vec3 a(0.0f);
    glm::vec3 b(0.0f);
    a[axis] = -reach;
    b[axis] = reach;
    return physics::capsule_shape(a, b, radius);
}

namespace detail {

inline constexpr int kCircleSegments = 24;

inline void circle(std::vector<Segment>& out, glm::vec3 centre, glm::vec3 u, glm::vec3 v, float radius) {
    glm::vec3 prev = centre + u * radius;
    for (int i = 1; i <= kCircleSegments; ++i) {
        const float t = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(kCircleSegments);
        const glm::vec3 next = centre + (u * std::cos(t) + v * std::sin(t)) * radius;
        out.push_back({prev, next});
        prev = next;
    }
}

// Any two unit vectors that, with `n`, are an orthonormal triple.
inline std::pair<glm::vec3, glm::vec3> perpendiculars(glm::vec3 n) {
    const glm::vec3 helper = std::fabs(n.x) < 0.9f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 u = glm::normalize(glm::cross(n, helper));
    return {u, glm::cross(n, u)};
}

// The edges of the convex hull of up to 16 points, drawn as polygons per face plane (a box's eight corners draw its twelve
// edges, not the diagonals of its faces). Deterministic: planes and edges come out in index order.
inline void hull_edges(std::vector<Segment>& out, const std::vector<glm::vec3>& p) {
    const std::size_t n = p.size();
    if (n == 1) {
        for (const glm::vec3 axis : {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)}) {
            out.push_back({p[0] - axis * 0.01f, p[0] + axis * 0.01f});
        }
        return;
    }
    if (n == 2) {
        out.push_back({p[0], p[1]});
        return;
    }
    glm::vec3 centroid(0.0f);
    float scale = 0.0f;
    for (const glm::vec3& q : p) centroid += q;
    centroid /= static_cast<float>(n);
    for (const glm::vec3& q : p) scale = std::max(scale, glm::length(q - centroid));
    const float eps = 1e-4f * std::max(scale, 1e-3f);

    std::map<std::array<int, 4>, std::pair<glm::vec3, float>> planes;  // quantized (normal, offset) -> exact plane
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            for (std::size_t k = j + 1; k < n; ++k) {
                glm::vec3 normal = glm::cross(p[j] - p[i], p[k] - p[i]);
                const float length = glm::length(normal);
                if (length < eps * eps) continue;  // collinear
                normal /= length;
                float offset = glm::dot(normal, p[i]);
                bool above = false;
                bool below = false;
                for (const glm::vec3& q : p) {
                    const float s = glm::dot(normal, q) - offset;
                    above = above || s > eps;
                    below = below || s < -eps;
                }
                if (above && below) continue;  // not a face of the hull
                if (glm::dot(normal, centroid) - offset > 0.0f) {  // outward: away from the centroid
                    normal = -normal;
                    offset = -offset;
                }
                const std::array<int, 4> key = {static_cast<int>(std::lround(normal.x * 1e3f)),
                                                static_cast<int>(std::lround(normal.y * 1e3f)),
                                                static_cast<int>(std::lround(normal.z * 1e3f)),
                                                static_cast<int>(std::lround(offset / std::max(scale, 1e-3f) * 1e3f))};
                planes.emplace(key, std::pair{normal, offset});
            }
        }
    }
    std::set<std::pair<std::size_t, std::size_t>> edges;
    for (const auto& [key, plane] : planes) {
        const auto& [normal, offset] = plane;
        std::vector<std::size_t> on;
        for (std::size_t i = 0; i < n; ++i) {
            if (std::fabs(glm::dot(normal, p[i]) - offset) <= eps) on.push_back(i);
        }
        if (on.size() < 3) continue;
        const auto [u, v] = perpendiculars(normal);
        const auto uv = [&](std::size_t i) { return glm::vec2(glm::dot(u, p[i]), glm::dot(v, p[i])); };
        std::sort(on.begin(), on.end(), [&](std::size_t a, std::size_t b) {
            const glm::vec2 pa = uv(a);
            const glm::vec2 pb = uv(b);
            return pa.x != pb.x ? pa.x < pb.x : pa.y != pb.y ? pa.y < pb.y : a < b;
        });
        // Andrew's monotone chain: the face's own outline, counter-clockwise in (u, v).
        const auto cross2 = [&](std::size_t o, std::size_t a, std::size_t b) {
            const glm::vec2 po = uv(o);
            const glm::vec2 pa = uv(a);
            const glm::vec2 pb = uv(b);
            return (pa.x - po.x) * (pb.y - po.y) - (pa.y - po.y) * (pb.x - po.x);
        };
        std::vector<std::size_t> chain;
        for (int pass = 0; pass < 2; ++pass) {
            const std::size_t start = chain.size();
            for (std::size_t m = 0; m < on.size(); ++m) {
                const std::size_t i = pass == 0 ? on[m] : on[on.size() - 1 - m];
                while (chain.size() >= start + 2 && cross2(chain[chain.size() - 2], chain.back(), i) <= 0.0f) {
                    chain.pop_back();
                }
                chain.push_back(i);
            }
            chain.pop_back();
        }
        for (std::size_t m = 0; m < chain.size(); ++m) {
            const std::size_t a = chain[m];
            const std::size_t b = chain[(m + 1) % chain.size()];
            if (a != b) edges.insert({std::min(a, b), std::max(a, b)});
        }
    }
    for (const auto& [a, b] : edges) out.push_back({p[a], p[b]});
}

}  // namespace detail

// A shape's wireframe, in its own (body) frame. A skin on a box or hull is not drawn (it is a contact margin, a few
// millimetres, not the shape).
[[nodiscard]] inline std::vector<Segment> shape_segments(const physics::ShapeDesc& shape) {
    using physics::ShapeKind;
    std::vector<Segment> out;
    switch (shape.kind) {
        case ShapeKind::none: break;
        case ShapeKind::sphere: {
            const glm::vec3 o(0.0f);
            detail::circle(out, o, glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), shape.radius);
            detail::circle(out, o, glm::vec3(0, 1, 0), glm::vec3(0, 0, 1), shape.radius);
            detail::circle(out, o, glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), shape.radius);
            break;
        }
        case ShapeKind::capsule: {
            const glm::vec3 d = shape.b - shape.a;
            const float length = glm::length(d);
            if (!(length > 0.0f)) break;
            const auto [u, v] = detail::perpendiculars(d / length);
            detail::circle(out, shape.a, u, v, shape.radius);
            detail::circle(out, shape.b, u, v, shape.radius);
            for (const glm::vec3 side : {u, -u, v, -v}) {
                out.push_back({shape.a + side * shape.radius, shape.b + side * shape.radius});
            }
            break;
        }
        case ShapeKind::box: {
            const glm::vec3 h = shape.half_extents;
            const auto corner = [&](int c) {
                return glm::vec3((c & 1) ? h.x : -h.x, (c & 2) ? h.y : -h.y, (c & 4) ? h.z : -h.z);
            };
            for (int c = 0; c < 8; ++c) {
                for (const int bit : {1, 2, 4}) {
                    if ((c & bit) == 0) out.push_back({corner(c), corner(c | bit)});
                }
            }
            break;
        }
        case ShapeKind::hull: detail::hull_edges(out, shape.points); break;
    }
    return out;
}

// The segments placed in the world by a body's pose (body_pose()).
[[nodiscard]] inline std::vector<Segment> placed(std::vector<Segment> segments, const FrameState& body) {
    for (Segment& s : segments) {
        s.a = body.pos + body.orient * s.a;
        s.b = body.pos + body.orient * s.b;
    }
    return segments;
}

// The line colour index the viewport gives a contact shape (ui_theme.hpp's axis_colour: 0-2 the gizmo's axes, 3 an airflow
// region's box, 4 this).
inline constexpr int kShapeLine = 4;

// The model's contact shape as the viewport draws it (L82): its wireframe placed at the vehicle whose DESIGN frame is `design`
// (the run's state while it runs, else the scene's start) and projected into a view of `width` x `height` pixels. An edge with
// an end behind the camera is left out; no shape, no lines.
[[nodiscard]] inline std::vector<GizmoLine> shape_view_lines(const vehicles::ModelType& model, const FrameState& design,
                                                            const render::Camera& cam, uint32_t width, uint32_t height) {
    std::vector<GizmoLine> lines;
    if (!model.shape) return lines;
    const FrameState body = body_pose(model, design.pos, design.orient);
    for (const Segment& s : placed(shape_segments(*model.shape), body)) {
        const std::optional<glm::vec2> a = project(cam, s.a, width, height);
        const std::optional<glm::vec2> b = project(cam, s.b, width, height);
        if (a && b) lines.push_back(GizmoLine{*a, *b, kShapeLine, false});
    }
    return lines;
}

}  // namespace spade::sandbox::editor
