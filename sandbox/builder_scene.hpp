// BuilderScene -- the sandbox's OBJECT MODEL, and every piece of maths the
// builder needs to place, pick and move things.
//
// WHY THIS IS A PURE HEADER WITH NO GLFW, NO GL AND NO IMGUI IN IT.
// SL15b rules the window "a deliberately dumb shell -- any logic that migrates
// into it is a defect, because it becomes untestable by construction." A
// builder is exactly the feature that tempts you to break that: picking,
// dragging and placing all *feel* like window code because that is where the
// mouse is. They are not. Picking is ray/solid intersection, placing is a
// ray/plane hit, and dragging is the same hit applied to a transform -- none
// of which needs a display, and all of which is wrong in ways a human staring
// at a viewport cannot reliably see. Every one of them lives here, where a
// test with no window can assert it against a hand-computed answer.
//
// The window's remaining job is genuinely dumb: turn mouse state into a ray,
// call one of these, and draw the panel.
//
// WHAT THE FRAME LOOP MUST NOT DO, AND WHY THE SPLIT BELOW EXISTS. The GPU
// renderer uploads GEOMETRY AND MATERIALS ONCE (GlRenderer::upload_scene) and
// refreshes only the per-instance transform buffers per frame -- that is the
// entire reason the render path is 159x faster than the CPU one. So:
//   * MOVING an object changes only its DrawItem transform  -> rebuild
//     `dynamics` every frame, which is cheap and is what the renderer expects.
//   * ADDING/REMOVING an object, or editing a COLOUR, changes the mesh or
//     material SET -> call upload_scene again, ONCE, on the edit.
// Conflating the two would either make colour edits invisible or re-upload the
// world every frame, and the second is a defect of the quiet kind: the window
// would run at a fraction of its frame rate for a reason no profile line names.

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/scene.hpp"
#include "render/target.hpp"

#include "orbit_camera.hpp"  // FrameInput -- also pure, also display-free

namespace spade::sandbox {

// The primitive set. Deliberately three: a builder that can only place one
// shape cannot show that mesh_index routing works, and every count past three
// is more mesh code rather than more capability.
enum class Shape : uint32_t { box = 0, sphere = 1, cylinder = 2 };

inline constexpr uint32_t kShapeCount = 3u;

[[nodiscard]] inline const char* shape_name(Shape s) noexcept {
    switch (s) {
        case Shape::box: return "box";
        case Shape::sphere: return "sphere";
        case Shape::cylinder: return "cylinder";
    }
    return "box";
}

// One placed object.
//
// WARNING: `scale` IS FULL EXTENT, NOT HALF-EXTENT, and the primitives below
// are built as UNIT shapes spanning [-0.5, 0.5] so that a scale of 1 means one
// metre on that axis. Stated here because the two conventions look identical
// in every screenshot and differ by exactly 2x in every number.
struct BuilderObject {
    std::string name;
    Shape shape = Shape::box;
    glm::vec3 position{0.0f, 0.5f, 0.0f};
    float yaw = 0.0f;   // radians about world +Y; the only rotation the UI exposes
    glm::vec3 scale{1.0f, 1.0f, 1.0f};
    glm::vec4 color{0.72f, 0.74f, 0.78f, 1.0f};

    // Physics settings. Carried on the object because the user asked to
    // "adjust physics or rendering settings" per object; nothing steps them
    // yet, and that absence is honest -- see BuilderScene::physics_running.
    bool dynamic = false;
    float mass = 1.0f;
    float restitution = 0.4f;
    float friction = 0.5f;
};

// The model. `selected` is an INDEX, not a pointer, because objects live in a
// vector that erase() reorders -- a pointer selection survives an erase as a
// dangling read, and an index selection survives it as a wrong selection,
// which is visible instead of fatal.
struct BuilderScene {
    std::vector<BuilderObject> objects;
    int selected = -1;   // -1 = nothing selected

    // Scene-wide render settings the inspector edits.
    bool grid = true;
    float sun_intensity = 1.0f;
    glm::vec3 sun_direction{0.37139067f, 0.74278135f, 0.55708601f};  // toward the sun: (0.4, 0.8, 0.6) normalised

    // Scene-wide physics settings.
    float gravity = -9.81f;
    bool physics_running = false;

    // ----- Tool state, and the panel's requests back to the frame loop ----
    //
    // THE PANEL DOES NOT MUTATE THE OBJECT LIST; IT RAISES A FLAG AND THE
    // FRAME LOOP ACTS ON IT. Erasing from `objects` while the panel is
    // iterating it is the classic version of this bug, and the version that
    // survives review is worse: the panel erases, `selected` still points at
    // the old index, and the next widget in the same frame reads a different
    // object than the one whose button was clicked. One owner for the list,
    // and it is the loop.
    Shape pending_shape = Shape::box;
    bool placing = true;      // true = click places, false = click selects
    bool materials_dirty = true;   // the material SET changed; re-upload once
    bool delete_request = false;
    bool duplicate_request = false;

    // Drag state. `dragging` is owned by the frame loop, not the panel.
    bool dragging = false;
    glm::vec3 drag_offset{0.0f};   // hit point -> object centre, at grab time

    [[nodiscard]] bool has_selection() const noexcept {
        return selected >= 0 && static_cast<size_t>(selected) < objects.size();
    }
    [[nodiscard]] BuilderObject* selected_object() noexcept {
        return has_selection() ? &objects[static_cast<size_t>(selected)] : nullptr;
    }
    [[nodiscard]] const BuilderObject* selected_object() const noexcept {
        return has_selection() ? &objects[static_cast<size_t>(selected)] : nullptr;
    }
};

// ---------------------------------------------------------------------------
// Rays
// ---------------------------------------------------------------------------

struct Ray {
    glm::vec3 origin{0.0f};
    glm::vec3 direction{0.0f, 0.0f, -1.0f};   // unit
};

// Screen pixel -> world ray.
//
// THE CONVENTION IS THE ENGINE'S AND IS NOT NEGOTIABLE HERE: camera space is
// right-handed with FORWARD = local -Z (raster_cpu.cpp's own derivation), and
// Camera::orientation maps local -> world. `px`/`py` are framebuffer pixels
// with the ORIGIN AT TOP-LEFT and +y DOWN, which is what every mouse API
// reports and the opposite of what NDC uses -- hence the flip on y and not on
// x. Getting that flip wrong produces a picker that is correct along one axis
// and mirrored along the other, which reads as "picking is flaky".
[[nodiscard]] inline Ray ray_from_screen(const render::Camera& camera, float px, float py,
                                         uint32_t width, uint32_t height) noexcept {
    const float w = static_cast<float>(width == 0u ? 1u : width);
    const float h = static_cast<float>(height == 0u ? 1u : height);
    const float tan_half = std::tan(camera.fov_y_radians * 0.5f);
    const float aspect = w / h;
    const float ndc_x = (2.0f * (px + 0.5f) / w - 1.0f) * aspect * tan_half;
    const float ndc_y = (1.0f - 2.0f * (py + 0.5f) / h) * tan_half;
    const glm::vec3 dir_cam = glm::normalize(glm::vec3(ndc_x, ndc_y, -1.0f));
    Ray r;
    r.origin = camera.position;
    r.direction = glm::normalize(camera.orientation * dir_cam);
    return r;
}

// Where a ray meets the horizontal plane y = ground_y.
//
// Returns nothing when the ray is parallel to the plane OR points away from
// it. THE SECOND CASE IS THE ONE THAT MATTERS: a ray aimed at the sky has a
// perfectly good algebraic intersection BEHIND the camera, and a placement
// tool that uses it drops objects behind you when you click on the horizon.
[[nodiscard]] inline std::optional<glm::vec3> ray_ground_hit(const Ray& ray,
                                                             float ground_y) noexcept {
    constexpr float kParallelEpsilon = 1e-6f;
    const float dy = ray.direction.y;
    if (std::fabs(dy) < kParallelEpsilon) {
        return std::nullopt;
    }
    // ⚠ `safe_dy` IS A NO-OP AT RUNTIME AND EXISTS FOR THE COMPILER. The guard
    // above already returned for every |dy| below the epsilon, so the ternary's
    // second arm is unreachable -- but MSVC's C4723 analysis does not propagate
    // a fabs() comparison to the divisor, and this target builds /W4 /WX, so
    // the straightforward form is a hard error rather than a warning.
    //
    // ⭐ AND THE THING THAT MADE IT FIRE IS WORTH KNOWING: it compiled clean
    // inside the sandbox and failed only in the TEST translation unit, because
    // the test constructs a ray with direction.y LITERALLY 0.0f to prove this
    // very guard works. The compiler inlines that constant, sees a division by
    // zero on the path the guard excludes, and reports it. *The case written to
    // show the guard is right is what made the compiler doubt it.*
    const float safe_dy = (dy >= kParallelEpsilon || dy <= -kParallelEpsilon) ? dy : kParallelEpsilon;
    const float t = (ground_y - ray.origin.y) / safe_dy;
    if (t <= 0.0f) {
        return std::nullopt;
    }
    return ray.origin + ray.direction * t;
}

// An object's local->world transform: translate * rotateY * scale.
[[nodiscard]] inline glm::mat4 object_transform(const BuilderObject& o) noexcept {
    glm::mat4 m = glm::translate(glm::mat4(1.0f), o.position);
    m = glm::rotate(m, o.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    m = glm::scale(m, o.scale);
    return m;
}

// Ray vs ONE object, in the object's own local space where every primitive is
// the same unit shape. Returns the distance along the ray, or nothing.
//
// WHY LOCAL SPACE RATHER THAN A WORLD-SPACE AABB: a world AABB of a rotated
// box is bigger than the box, so picking would select an object you can see
// you did not click on. Transforming the ray costs one inverse and makes the
// test exact for the box and the cylinder, and exact for the sphere under
// uniform scale.
[[nodiscard]] inline std::optional<float> ray_object_hit(const Ray& ray,
                                                         const BuilderObject& o) noexcept {
    const glm::mat4 inv = glm::inverse(object_transform(o));
    const glm::vec3 lo = glm::vec3(inv * glm::vec4(ray.origin, 1.0f));
    const glm::vec3 ld = glm::vec3(inv * glm::vec4(ray.direction, 0.0f));
    constexpr float kEps = 1e-8f;

    float t_hit = -1.0f;
    if (o.shape == Shape::sphere) {
        // Unit sphere, radius 0.5, centred at the origin.
        const float a = glm::dot(ld, ld);
        if (a < kEps) {
            return std::nullopt;
        }
        const float b = 2.0f * glm::dot(lo, ld);
        const float c = glm::dot(lo, lo) - 0.25f;
        const float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) {
            return std::nullopt;
        }
        const float sq = std::sqrt(disc);
        const float t0 = (-b - sq) / (2.0f * a);
        const float t1 = (-b + sq) / (2.0f * a);
        t_hit = (t0 > 0.0f) ? t0 : t1;
    } else {
        // Box and cylinder both start from the same [-0.5, 0.5] Y slab; the
        // cylinder then replaces the x/z slabs with a radial test. Sharing the
        // Y slab is deliberate -- the cap logic is identical and a second copy
        // is a second thing to get wrong.
        float t_near = -1e30f, t_far = 1e30f;
        if (std::fabs(ld.y) < kEps) {
            if (lo.y < -0.5f || lo.y > 0.5f) {
                return std::nullopt;
            }
        } else {
            float ta = (-0.5f - lo.y) / ld.y;
            float tb = (0.5f - lo.y) / ld.y;
            if (ta > tb) {
                const float s = ta;
                ta = tb;
                tb = s;
            }
            t_near = ta > t_near ? ta : t_near;
            t_far = tb < t_far ? tb : t_far;
            if (t_near > t_far) {
                return std::nullopt;
            }
        }
        if (o.shape == Shape::cylinder) {
            // Infinite cylinder of radius 0.5 about the local Y axis, then
            // clipped by the Y slab already accumulated above.
            const float a = ld.x * ld.x + ld.z * ld.z;
            const float b = 2.0f * (lo.x * ld.x + lo.z * ld.z);
            const float c = lo.x * lo.x + lo.z * lo.z - 0.25f;
            if (a < kEps) {
                if (c > 0.0f) {
                    return std::nullopt;   // parallel to the axis and outside
                }
            } else {
                const float disc = b * b - 4.0f * a * c;
                if (disc < 0.0f) {
                    return std::nullopt;
                }
                const float sq = std::sqrt(disc);
                float ta = (-b - sq) / (2.0f * a);
                float tb = (-b + sq) / (2.0f * a);
                if (ta > tb) {
                    const float s = ta;
                    ta = tb;
                    tb = s;
                }
                t_near = ta > t_near ? ta : t_near;
                t_far = tb < t_far ? tb : t_far;
                if (t_near > t_far) {
                    return std::nullopt;
                }
            }
        } else {
            // X and Z slabs for the box.
            const float o_xz[2] = {lo.x, lo.z};
            const float d_xz[2] = {ld.x, ld.z};
            for (int i = 0; i < 2; ++i) {
                if (std::fabs(d_xz[i]) < kEps) {
                    if (o_xz[i] < -0.5f || o_xz[i] > 0.5f) {
                        return std::nullopt;
                    }
                    continue;
                }
                float ta = (-0.5f - o_xz[i]) / d_xz[i];
                float tb = (0.5f - o_xz[i]) / d_xz[i];
                if (ta > tb) {
                    const float s = ta;
                    ta = tb;
                    tb = s;
                }
                t_near = ta > t_near ? ta : t_near;
                t_far = tb < t_far ? tb : t_far;
                if (t_near > t_far) {
                    return std::nullopt;
                }
            }
        }
        t_hit = (t_near > 0.0f) ? t_near : t_far;
    }
    if (t_hit <= 0.0f) {
        return std::nullopt;
    }
    // BACK TO WORLD DISTANCE. The local ray direction is NOT unit length under
    // a non-uniform scale, so `t` from the local test is in local parameter
    // units. Because the local ray was formed by transforming the WORLD origin
    // and direction by the same inverse, the parameter is shared: the same t
    // indexes the world ray. Returning it directly is therefore correct, and
    // this comment exists because it looks like it should not be.
    return t_hit;
}

// The nearest object under the ray, or -1.
//
// NEAREST, NOT FIRST. Iterating and returning the first hit makes selection
// depend on insertion order, so clicking an object that sits behind another
// selects the wrong one -- and it does so only when two objects overlap on
// screen, which is exactly when a human cannot tell the tool is wrong.
[[nodiscard]] inline int pick_object(const BuilderScene& scene, const Ray& ray) noexcept {
    int best = -1;
    float best_t = 1e30f;
    for (size_t i = 0; i < scene.objects.size(); ++i) {
        const std::optional<float> t = ray_object_hit(ray, scene.objects[i]);
        if (t && *t < best_t) {
            best_t = *t;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Primitive geometry -- unit shapes spanning [-0.5, 0.5], flat-shaded.
//
// Built here rather than pulled from the engine's tessellator on purpose: SL2b
// limits this target to headers the engine INSTALLS, and a builder that can
// only place shapes the SDF tessellator happens to expose is a builder whose
// palette is decided somewhere else. These are small, exact, and theirs.
// ---------------------------------------------------------------------------

namespace detail {

inline void push_tri(render::MeshData& m, const glm::vec3& a, const glm::vec3& b,
                     const glm::vec3& c) {
    const glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));
    const uint32_t base = static_cast<uint32_t>(m.positions.size());
    m.positions.push_back(a);
    m.positions.push_back(b);
    m.positions.push_back(c);
    m.normals.push_back(n);
    m.normals.push_back(n);
    m.normals.push_back(n);
    m.indices.push_back(base);
    m.indices.push_back(base + 1u);
    m.indices.push_back(base + 2u);
}

inline void finish(render::MeshData& m) {
    m.submesh_first_index = {0u};
    m.submesh_index_count = {static_cast<uint32_t>(m.indices.size())};
    m.submesh_material = {0u};
}

}  // namespace detail

// The size parameters exist so a LIT part can be built at its real dimensions
// and placed with rotation and translation only: both shading paths transform
// normals by mat3(model) with no inverse-transpose, so a non-uniformly scaled
// box shades skewed. The defaults are the unit meshes the builder scales.
[[nodiscard]] inline render::MeshData make_box_mesh(const glm::vec3& half = glm::vec3(0.5f)) {
    render::MeshData m;
    const float x = half.x, y = half.y, z = half.z;
    const glm::vec3 p[8] = {
        {-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z},
        {-x, -y, z},  {x, -y, z},  {x, y, z},  {-x, y, z},
    };
    // Counter-clockwise seen from outside.
    const int faces[6][4] = {
        {4, 5, 6, 7},  // +z
        {1, 0, 3, 2},  // -z
        {5, 1, 2, 6},  // +x
        {0, 4, 7, 3},  // -x
        {3, 7, 6, 2},  // +y
        {0, 1, 5, 4},  // -y
    };
    for (const auto& f : faces) {
        detail::push_tri(m, p[f[0]], p[f[1]], p[f[2]]);
        detail::push_tri(m, p[f[0]], p[f[2]], p[f[3]]);
    }
    detail::finish(m);
    return m;
}

[[nodiscard]] inline render::MeshData make_sphere_mesh(uint32_t rings = 12u,
                                                       uint32_t sectors = 18u) {
    render::MeshData m;
    const float r = 0.5f;
    const float pi = 3.14159265358979323846f;
    auto at = [&](uint32_t ri, uint32_t si) {
        const float phi = pi * static_cast<float>(ri) / static_cast<float>(rings);
        const float theta = 2.0f * pi * static_cast<float>(si) / static_cast<float>(sectors);
        return glm::vec3(r * std::sin(phi) * std::cos(theta), r * std::cos(phi),
                         r * std::sin(phi) * std::sin(theta));
    };
    for (uint32_t ri = 0; ri < rings; ++ri) {
        for (uint32_t si = 0; si < sectors; ++si) {
            const glm::vec3 a = at(ri, si);
            const glm::vec3 b = at(ri + 1u, si);
            const glm::vec3 c = at(ri + 1u, si + 1u);
            const glm::vec3 d = at(ri, si + 1u);
            // Counter-clockwise seen from OUTSIDE, like the box. At the north
            // pole a == d and at the south pole b == c, so the triangle that
            // collapses there is the one skipped -- the two conditions were
            // once the other way round, which left a hole at each pole and a
            // zero-area triangle with a NaN normal in its place.
            if (ri + 1u != rings) {
                detail::push_tri(m, a, c, b);
            }
            if (ri != 0u) {
                detail::push_tri(m, a, d, c);
            }
        }
    }
    detail::finish(m);
    return m;
}

[[nodiscard]] inline render::MeshData make_cylinder_mesh(uint32_t sectors = 20u, float radius = 0.5f,
                                                         float half_height = 0.5f) {
    render::MeshData m;
    const float r = radius, h = half_height;
    const float pi = 3.14159265358979323846f;
    auto ring = [&](uint32_t si, float y) {
        const float t = 2.0f * pi * static_cast<float>(si) / static_cast<float>(sectors);
        return glm::vec3(r * std::cos(t), y, r * std::sin(t));
    };
    for (uint32_t si = 0; si < sectors; ++si) {
        const glm::vec3 a = ring(si, -h), b = ring(si + 1u, -h);
        const glm::vec3 c = ring(si + 1u, h), d = ring(si, h);
        // Counter-clockwise seen from OUTSIDE, like the box. The ring runs
        // from +X toward +Z, so the outward side triangles are (a, c, b) and
        // (a, d, c).
        detail::push_tri(m, a, c, b);
        detail::push_tri(m, a, d, c);
        detail::push_tri(m, glm::vec3(0.0f, h, 0.0f), c, d);     // top cap, normal +Y
        detail::push_tri(m, glm::vec3(0.0f, -h, 0.0f), a, b);    // bottom cap, normal -Y
    }
    detail::finish(m);
    return m;
}

// ---------------------------------------------------------------------------
// Model -> RenderScene
// ---------------------------------------------------------------------------

// Where the builder's own meshes and materials start inside a RenderScene that
// already holds a world's ground. Returned rather than assumed, because the
// base scene's own counts are not this module's to predict.
struct BuilderBinding {
    uint32_t mesh_base = 0;
    uint32_t material_base = 0;
};

// Appends the three primitives to a scene that already exists. Call ONCE.
[[nodiscard]] inline BuilderBinding bind_builder_meshes(render::RenderScene& scene) {
    BuilderBinding b;
    b.mesh_base = static_cast<uint32_t>(scene.meshes.size());
    b.material_base = static_cast<uint32_t>(scene.materials.size());
    scene.meshes.push_back(make_box_mesh());
    scene.meshes.push_back(make_sphere_mesh());
    scene.meshes.push_back(make_cylinder_mesh());
    return b;
}

// Rewrites the material palette for the builder's objects. Call ONLY when the
// object set or a colour changed -- it is followed by an upload_scene(), which
// re-uploads geometry too.
inline void sync_builder_materials(const BuilderScene& model, const BuilderBinding& bind,
                                   render::RenderScene& scene) {
    scene.materials.resize(bind.material_base);
    for (const BuilderObject& o : model.objects) {
        render::Material m;
        m.base_color = o.color;
        m.shading = 0u;   // lambert
        scene.materials.push_back(m);
    }
    if (scene.materials.empty()) {
        scene.materials.push_back(render::Material{});
    }
}

// Rebuilds the per-frame draw list. Cheap, and safe to call every frame.
inline void rebuild_builder_dynamics(const BuilderScene& model, const BuilderBinding& bind,
                                     render::RenderScene& scene) {
    scene.dynamics.clear();
    scene.dynamics.reserve(model.objects.size());
    for (size_t i = 0; i < model.objects.size(); ++i) {
        const BuilderObject& o = model.objects[i];
        render::DrawItem item;
        item.mesh_index = bind.mesh_base + static_cast<uint32_t>(o.shape);
        item.local_to_world = object_transform(o);
        item.material_override = bind.material_base + static_cast<uint32_t>(i);
        scene.dynamics.push_back(item);
    }
}

// A new object, resting ON the ground at `hit` rather than centred in it.
//
// THE HALF-HEIGHT OFFSET IS THE WHOLE DIFFERENCE BETWEEN A TOOL THAT FEELS
// RIGHT AND ONE THAT DOES NOT. Placing an object's CENTRE at the ray/ground
// hit buries half of it under the floor, every time, and the user's first
// action is then always to fix the height by hand.
[[nodiscard]] inline BuilderObject make_object_at(Shape shape, const glm::vec3& hit, int ordinal) {
    BuilderObject o;
    o.shape = shape;
    o.scale = glm::vec3(1.0f);
    o.position = glm::vec3(hit.x, hit.y + o.scale.y * 0.5f, hit.z);
    o.name = std::string(shape_name(shape)) + "_" + std::to_string(ordinal);
    // A palette that walks rather than a single grey, so two adjacent objects
    // are distinguishable without opening the inspector.
    static const glm::vec4 kPalette[6] = {
        {0.82f, 0.36f, 0.32f, 1.0f}, {0.38f, 0.62f, 0.85f, 1.0f},
        {0.46f, 0.76f, 0.45f, 1.0f}, {0.90f, 0.72f, 0.32f, 1.0f},
        {0.68f, 0.48f, 0.82f, 1.0f}, {0.35f, 0.75f, 0.74f, 1.0f},
    };
    o.color = kPalette[static_cast<size_t>(ordinal) % 6u];
    return o;
}

// ---------------------------------------------------------------------------
// ONE FRAME OF BUILDER INPUT -- the whole interaction, as a pure function
//
// THIS IS THE FUNCTION SL15b IS ABOUT. Placing, picking, dragging and deleting
// are the builder's entire behaviour, and every one of them is decided here,
// from a FrameInput and a Camera, with no window in scope. The frame loop
// calls it and applies the result; a test calls it with a synthesised
// FrameInput and asserts the model moved the way it should.
//
// A version of this written inside the window loop would be equally correct
// and completely unverifiable, and "it looked right when I dragged it" is not
// a claim anyone can check or re-run.
// ---------------------------------------------------------------------------

// What the frame loop must do after applying input.
struct BuilderFrameResult {
    // The mesh/material SET changed, so the GPU renderer's one-time upload is
    // stale. Distinct from a transform change, which needs nothing.
    bool needs_upload = false;
    bool selection_changed = false;
};

inline BuilderFrameResult apply_builder_input(BuilderScene& model, const FrameInput& in,
                                              const render::Camera& camera, uint32_t width,
                                              uint32_t height, float ground_y) {
    BuilderFrameResult out;
    const int before = model.selected;

    // Panel-driven requests first, and they are handled HERE rather than in
    // the panel for the reason BuilderScene's own comment gives: one owner for
    // the object list.
    if (model.delete_request || (in.delete_pressed && !in.ui_captured_keyboard)) {
        model.delete_request = false;
        if (model.has_selection()) {
            model.objects.erase(model.objects.begin() + model.selected);
            // CLAMP RATHER THAN CLEAR. Deleting the middle of a list and
            // landing on "nothing selected" makes deleting several objects in
            // a row need a re-click between each one.
            if (model.objects.empty()) {
                model.selected = -1;
            } else if (model.selected >= static_cast<int>(model.objects.size())) {
                model.selected = static_cast<int>(model.objects.size()) - 1;
            }
            model.dragging = false;
            model.materials_dirty = true;
        }
    }
    if (model.duplicate_request || (in.duplicate_pressed && !in.ui_captured_keyboard)) {
        model.duplicate_request = false;
        if (model.has_selection()) {
            BuilderObject copy = model.objects[static_cast<size_t>(model.selected)];
            // OFFSET SO THE COPY IS VISIBLE. A duplicate placed exactly on its
            // original looks like nothing happened, and the user presses the
            // key again.
            copy.position.x += copy.scale.x;
            copy.name += "_copy";
            model.objects.push_back(copy);
            model.selected = static_cast<int>(model.objects.size()) - 1;
            model.materials_dirty = true;
        }
    }

    // A release always ends a drag, even when the pointer finished over a
    // panel -- see the note beside left_release in orbit_camera.hpp.
    if (in.left_release) {
        model.dragging = false;
    }

    if (!in.ui_captured_mouse) {
        const Ray ray = ray_from_screen(camera, in.mouse_x, in.mouse_y, width, height);

        if (in.left_click) {
            const int hit = pick_object(model, ray);
            if (hit >= 0) {
                // AN OBJECT UNDER THE CURSOR ALWAYS WINS, EVEN IN PLACEMENT
                // MODE. Placing a new object on top of an existing one is
                // almost never what a click on that object meant, and the
                // alternative -- having to leave placement mode to select --
                // is the friction that makes a builder tiring.
                model.selected = hit;
                model.dragging = true;
                if (const std::optional<glm::vec3> g = ray_ground_hit(ray, ground_y)) {
                    // GRAB OFFSET, SO THE OBJECT DOES NOT JUMP. Without it the
                    // object's centre snaps to the cursor on the first frame
                    // of every drag, which reads as the tool being imprecise.
                    model.drag_offset = model.objects[static_cast<size_t>(hit)].position - *g;
                    model.drag_offset.y = 0.0f;
                } else {
                    model.drag_offset = glm::vec3(0.0f);
                }
            } else if (model.placing) {
                if (const std::optional<glm::vec3> g = ray_ground_hit(ray, ground_y)) {
                    model.objects.push_back(make_object_at(model.pending_shape, *g,
                                                           static_cast<int>(model.objects.size())));
                    model.selected = static_cast<int>(model.objects.size()) - 1;
                    model.materials_dirty = true;
                }
            } else {
                model.selected = -1;   // click on empty space clears
            }
        } else if (in.left_down && model.dragging && model.has_selection()) {
            if (const std::optional<glm::vec3> g = ray_ground_hit(ray, ground_y)) {
                BuilderObject& o = model.objects[static_cast<size_t>(model.selected)];
                const glm::vec3 p = *g + model.drag_offset;
                // DRAGS IN THE GROUND PLANE ONLY, AND Y IS LEFT ALONE. A drag
                // that also changes height needs a second axis of input the
                // mouse does not have, and guessing one moves the object
                // somewhere the user cannot see.
                o.position.x = p.x;
                o.position.z = p.z;
            }
        }
    }

    if (model.materials_dirty) {
        out.needs_upload = true;
        model.materials_dirty = false;
    }
    out.selection_changed = (model.selected != before);
    return out;
}

}  // namespace spade::sandbox
