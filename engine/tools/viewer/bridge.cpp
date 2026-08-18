// ---------------------------------------------------------------------------
// bridge.cpp -- the ONLY translation unit in spade_viewer that includes both
// v1 (Spade/Spade.hpp, and everything it pulls in: glad, GLFW, windows.h) and
// v2 (sim/simulation.hpp, world/sdf.hpp, ...) headers.
//
// WHY THAT ISOLATION EXISTS. spade_viewer links spade_warnings (/W4 /WX,
// engine/CMakeLists.txt) like every other v2 target -- but v1's own `Spade`
// library target does NOT (spade/src/CMakeLists.txt), and v1 is frozen, so
// there is no fixing a v1 header that does not survive /W4 /WX. Rather than
// weaken the whole spade_viewer target's warning policy for main.cpp and
// scenes.cpp too (both of which are v1-free and DO compile clean under it),
// v1's includes are confined to this one file, and engine/CMakeLists.txt
// gives bridge.cpp its own, relaxed per-source compile options.
//
// Everything v1-shaped lives here: the Universe/Engine/MeshComponent
// construction, the SDF-primitive -> v1-mesh approximation, and the render
// loop. bridge.hpp's Scene/BodyPlacement/Viewer types are the only surface
// the rest of the tool (main.cpp, scenes.cpp) sees.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// <windows.h> FIRST, DELIBERATELY -- AHEAD OF EVERYTHING ELSE IN THIS FILE,
// INCLUDING bridge.hpp (S5 T9 /WX ticket). v1's Spade/Core/Engine.hpp
// (spade/include/**, FROZEN -- not editable) includes <glad/glad.h> BEFORE
// its own <windows.h>, and minwindef.h's APIENTRY definition then collides
// with glad.h's own -- warning C4005 "macro redefinition", fatal under /WX
// (engine/CMakeLists.txt's spade_viewer target comment has the fuller
// history). The fix does not require touching the frozen v1 header: glad.h
// guards its own definition (`#if defined(_WIN32) && !defined(APIENTRY) &&
// ...`), so if <windows.h> has ALREADY defined APIENTRY (via minwindef.h) by
// the time glad.h runs -- which is what including it here, before anything
// that reaches glad.h transitively, guarantees -- glad.h's guard skips its
// own definition and there is nothing left to collide.
//
// OUTCOME: this alone WAS sufficient. bridge.cpp now compiles clean under
// the target's full /W4 /WX, and engine/CMakeLists.txt's spade_viewer block
// no longer carries a source-level /WX- override for this file at all (S5
// T9's report has the literal before/after build output).
#include <windows.h>

#include "bridge.hpp"

#include <cmath>
#include <cstdio>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>  // glm::rotation (shortest-arc quat) -- GTX, needs
                                   // GLM_ENABLE_EXPERIMENTAL (defined for the whole
                                   // spade build by the root CMakeLists.txt)

#include <Spade/Spade.hpp>

#include "physics/contacts.hpp"
#include "world/sdf.hpp"

namespace spade::viewer {
namespace {

// ---------------------------------------------------------------------------
// Frame pacing (tools/ wall-clock exemption, global-constraints.md). The v2
// physics dt/substep decomposition itself is untouched by any of this -- it
// is decided ONCE, here, and Simulation::create() pins it for the life of the
// Simulation; nothing below ever varies it per frame.
//
// 4 ms outer step / 4 substeps -> a 1 ms (1 kHz) effective substep, close to
// the coordinator's suggested ~1/240 s (4.1667 ms, which is not an exact
// nanosecond count -- dt_ns must divide evenly by substeps, see
// Simulation::create()'s doc comment) and landing exactly on the 1 kHz
// reference rate the engine's own docs cite repeatedly (physics/contacts.hpp,
// world/medium.hpp) -- a rounder, more defensible choice than an inexact
// 1/240 s would have been.
constexpr uint64_t kStepDtNs = 4'000'000;
constexpr uint32_t kSubsteps = 4;
constexpr float kStepDtSeconds = 0.004f;

// Debt cap (coordinator resolution): a debugger pause or a slow frame must
// not make the accumulator demand years of steps on the next tick.
constexpr float kMaxAccumulatedDebtSeconds = 0.25f;

// ---------------------------------------------------------------------------
// SDF primitive -> v1 mesh instance, the strangler bridge's core mapping.
// ---------------------------------------------------------------------------

// A primitive node's WORLD-space pose, recovered from its (pre-inverted, per
// world/sdf.hpp) SdfTransform.
//
// world/builder.cpp's add_transform() builds world_to_local as
//   L = mat3(world_to_local) = transpose(R) / scale         (R the pose's rotation)
//   c = world_to_local[3].xyz = -(L * position)
// so, since R is orthogonal ((R^T)^-1 == R):
//   R    = scale * transpose(L)            -- this is `local_to_world_rotation` below: the
//                                              scale cancels out of scale * transpose(L), so
//                                              it recovers the PURE, unscaled authoring
//                                              rotation, not a scaled one.
//   L^-1 = scale * R                       -- the full local-to-world map (rotate, then
//                                              scale) is a SEPARATE quantity from R above --
//                                              it still needs `scale` applied.
//   position = -(L^-1 * c) = -(scale * R * c) = -(scale * local_to_world_rotation * c)
// Get this wrong (as an earlier version of this function did) and both outputs are corrupted
// for any non-unit scale: dropping the `scale *` on position silently divides the recovered
// position by `scale`, and adding a spurious `/ scale` on the rotation re-divides an
// already-correct, unscaled rotation matrix a second time.
struct WorldSpacePose {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    float scale = 1.0f;
};

[[nodiscard]] WorldSpacePose pose_of(const spade::SdfTransform& t) noexcept {
    const glm::mat3 local_to_world_rotation = t.scale * glm::transpose(glm::mat3(t.world_to_local));
    const glm::vec3 c(t.world_to_local[3]);

    WorldSpacePose pose;
    pose.position = -(t.scale * (local_to_world_rotation * c));
    pose.rotation = glm::quat_cast(local_to_world_rotation);
    pose.scale = t.scale;
    return pose;
}

// Which primitive LEAF nodes should be rendered.
//
// A literal per-leaf walk (every primitive drawn as its own approximate
// shape -- box/sphere/torus/plane below) is the mapping the brief asks for,
// EXCEPT for a subtract()'s SECOND operand: that primitive is the cutting
// tool carving a cavity out of the first, and rendering it as its own opaque
// shape would draw solid geometry exactly where the cavity is meant to read
// as empty (the shower scene's bowl is subtract(box, sphere) -- a rendered
// sphere there would occlude the very depression it carves, which is actively
// misleading rather than merely approximate). union/intersect/smooth_union
// are not sculpting a branch into invisibility, so both operands stay
// rendered under those ops.
//
// Implemented as the SAME postfix stack walk world/sdf.cpp's eval() uses,
// except each stack slot holds the SET of leaf node indices still "in" the
// visible solid rather than a distance.
[[nodiscard]] std::vector<uint32_t> visible_leaves(const spade::SdfProgram& sdf) {
    std::vector<std::vector<uint32_t>> stack;
    for (uint32_t i = 0; i < static_cast<uint32_t>(sdf.nodes.size()); ++i) {
        const spade::SdfNode& node = sdf.nodes[i];
        if (node.op == static_cast<uint32_t>(spade::SdfOp::none)) {
            stack.push_back(std::vector<uint32_t>{i});
            continue;
        }
        // sdf has passed SdfProgram::validate() (every scene builds through
        // WorldBuilder::build(), which calls it), so >= 2 operands are
        // guaranteed here -- same precondition eval() relies on.
        std::vector<uint32_t> b = std::move(stack.back());
        stack.pop_back();
        std::vector<uint32_t> a = std::move(stack.back());
        stack.pop_back();

        if (static_cast<spade::SdfOp>(node.op) != spade::SdfOp::subtract) {
            a.insert(a.end(), b.begin(), b.end());
        }
        // subtract: `b`'s leaves are dropped -- `a` alone carries forward.
        stack.push_back(std::move(a));
    }
    return stack.empty() ? std::vector<uint32_t>{} : stack.back();
}

// Appends one instance (transform + neutral motion + a flat color) to a v1
// MeshComponent's parallel instance arrays.
void push_instance(Spade::MeshComponent& mesh, const glm::vec3& position, const glm::quat& rotation,
                    const glm::vec3& scale, const glm::vec4& color) {
    Spade::Transform transform;
    transform.position = position;
    transform.rotation = rotation;
    transform.scale = scale;
    mesh.instanceTransforms.push_back(transform);

    mesh.instanceMotions.push_back(Spade::Motion{});

    Spade::Material material;
    material.color = color;
    mesh.instanceMaterials.push_back(material);
}

void push_box(Spade::MeshComponent& mesh, const glm::vec3& position, const glm::quat& rotation,
              const glm::vec3& full_extent, const glm::vec4& color) {
    push_instance(mesh, position, rotation, full_extent, color);
}

void push_sphere(Spade::MeshComponent& mesh, const glm::vec3& position, float radius, const glm::vec4& color) {
    push_instance(mesh, position, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(radius), color);
}

// plane -> a large, finite, thin box (brief: "plane -> large thin cube").
// A plane primitive is an infinite half-space with no rendered extent of its
// own; `half_extent`/`thickness` are the Scene's rendering-only knobs for how
// big a slab to draw, centered/oriented at the node's own pose.
void push_plane_as_box(Spade::MeshComponent& mesh, const spade::SdfNode& node, const WorldSpacePose& pose,
                        float half_extent, float thickness, const glm::vec4& color) {
    const glm::vec3 n_local(node.params);
    const float offset = node.params.w;

    const glm::vec3 n_world = glm::normalize(pose.rotation * n_local);
    const glm::vec3 point_local = n_local * offset;
    const glm::vec3 point_world = pose.position + pose.rotation * (point_local * pose.scale);

    // Shortest-arc rotation from the cube mesh's own +Y (its "top face"
    // normal, Primitives.cpp's GenerateCube) onto the plane's world normal.
    const glm::quat box_rotation = glm::rotation(glm::vec3(0.0f, 1.0f, 0.0f), n_world);
    // Center the slab HALF A THICKNESS below the surface point along the
    // normal, so its top face lies exactly on the plane rather than straddling it.
    const glm::vec3 box_center = point_world - n_world * (thickness * 0.5f);

    push_box(mesh, box_center, box_rotation, glm::vec3(half_extent * 2.0f, thickness, half_extent * 2.0f), color);
}

// torus -> a ring of spheres approximating the tube (brief: "approximate
// with a ring of spheres ... document the approximation"). Each little
// sphere sits on the major-radius circle in the node's local XZ plane (the
// torus's own local frame, world/sdf.hpp) with the minor radius, mapped to
// world space by the node's pose. Individual segment orientation is not
// modeled (spheres are rotation-invariant), and the tube's circular
// cross-section is only sampled at `kSegments` points around the ring rather
// than continuously -- a coarse but honest stand-in for contact geometry
// that is otherwise exact (the physics uses the real analytic torus SDF; only
// the render is approximated).
void push_torus_ring(Spade::MeshComponent& mesh, const spade::SdfNode& node, const WorldSpacePose& pose,
                      const glm::vec4& color) {
    constexpr int kSegments = 20;
    const float major_radius = node.params.x;
    const float minor_radius = node.params.y;

    for (int i = 0; i < kSegments; ++i) {
        const float angle = (glm::two_pi<float>() * static_cast<float>(i)) / static_cast<float>(kSegments);
        const glm::vec3 local_point(major_radius * std::cos(angle), 0.0f, major_radius * std::sin(angle));
        const glm::vec3 world_point = pose.position + pose.rotation * (local_point * pose.scale);
        push_sphere(mesh, world_point, minor_radius * pose.scale, color);
    }
}

// Walks one world's SDF program's visible leaves and appends the
// corresponding approximate instance to `boxes` (box, plane) or `spheres`
// (sphere, torus). cylinder/capsule/heightfield are not rendered: heightfield
// is explicitly out of scope (brief: "heightfield skipped"), and
// cylinder/capsule have no demo geometry exercising them -- silently drawing
// nothing is the honest choice for a shape no scene here ever uses.
void add_static_geometry(const spade::SdfProgram& sdf, Spade::MeshComponent& boxes, Spade::MeshComponent& spheres,
                          const glm::vec4& color, float ground_half_extent, float ground_thickness) {
    for (uint32_t node_index : visible_leaves(sdf)) {
        const spade::SdfNode& node = sdf.nodes[node_index];
        const spade::SdfTransform& transform = sdf.transforms[node.transform];
        const WorldSpacePose pose = pose_of(transform);

        switch (static_cast<spade::SdfPrim>(node.kind)) {
            case spade::SdfPrim::box: {
                const glm::vec3 half_extents(node.params);
                push_box(boxes, pose.position, pose.rotation,
                         glm::vec3(2.0f * half_extents.x, 2.0f * half_extents.y, 2.0f * half_extents.z) *
                             pose.scale,
                         color);
                break;
            }
            case spade::SdfPrim::plane:
                push_plane_as_box(boxes, node, pose, ground_half_extent, ground_thickness, color);
                break;
            case spade::SdfPrim::sphere:
                push_sphere(spheres, pose.position, node.params.x * pose.scale, color);
                break;
            case spade::SdfPrim::torus:
                push_torus_ring(spheres, node, pose, color);
                break;
            case spade::SdfPrim::cylinder:
            case spade::SdfPrim::capsule:
            case spade::SdfPrim::heightfield:
                break;  // not used by any demo scene; see the function comment
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Viewer::Impl
// ---------------------------------------------------------------------------

struct Viewer::Impl {
    Scene scene;

    // Declaration order is destruction order (reverse): `universe` must be
    // destroyed BEFORE `engine` (a MeshComponent's destructor calls
    // glDeleteVertexArrays/Buffers, which needs the GL context `engine` owns
    // to still be current), so `engine` is declared first.
    Spade::Engine engine;
    Spade::Universe universe;

    std::optional<spade::Simulation> sim;
    uint32_t world_count = 0;

    // The one MeshComponent whose instance transforms change every frame.
    // Stable for the Viewer's whole lifetime: fetched once, AFTER every
    // MeshComponent has been added to `universe` (see the constructor's
    // comment on ComponentPool<T> pointer stability).
    Spade::MeshComponent* dynamic_mesh = nullptr;

    // Every vehicle Task 20's Scene::vehicles spawned, in that same order --
    // what scene.command_hook is handed each step. Empty for every
    // pre-Task-20 scene (drop/bounce/shower/gate spawn no vehicles).
    std::vector<spade::VehicleRef> vehicle_refs;

    // The camera's TransformComponent (Task 20's orbit). Stable for the
    // Viewer's whole lifetime for the same reason `dynamic_mesh` is: fetched
    // once, and nothing else ever adds a TransformComponent afterward (only
    // the camera entity gets one -- see the constructor).
    Spade::TransformComponent* camera_transform = nullptr;

    float accumulator_s = 0.0f;

    // FPS counter (user amendment A1) rate-limit: seconds accumulated since
    // the last console print. See run()'s comment for why this is a SEPARATE
    // accumulator from `accumulator_s` above rather than reusing it.
    float fps_print_timer_s = 0.0f;

    // Camera orbit (Task 20) elapsed wall-clock seconds -- its OWN
    // accumulator for the same reason `fps_print_timer_s` is separate from
    // `accumulator_s`: orbit motion is pure presentation and must not
    // perturb, or be perturbed by, the fixed-step physics accumulator.
    float camera_orbit_elapsed_s = 0.0f;

    // The compute backend this Viewer's Simulation runs on (S6 Task 6). Held
    // so the FPS line can name it -- a demo whose whole point is "this is the
    // GPU stepping it" must say which path it took, or the user is taking the
    // command line's word for it.
    spade::compute::BackendDesc backend{};

    Impl(Scene s, spade::compute::BackendDesc desc);
    void run();
    void step_and_sync();
};

Viewer::Impl::Impl(Scene s, spade::compute::BackendDesc desc) : scene(std::move(s)), backend(desc) {
    // --- v2: build the Simulation, spawn every body at t=0 ----------------
    //
    // `backend` (S6 Task 6) is the ONLY thing that differs between a cpu run
    // and a vulkan one: the scene, the spawns, the step decomposition and the
    // render loop are identical, which is what makes `demo.ps1 -Scene bounce
    // -Backend vulkan` a demonstration of the PORT rather than of a second
    // code path.
    //
    // THROUGH S6 TASK 7 a world set the vulkan backend could not step
    // correctly was refused by Simulation::create() itself, with
    // Code::unavailable naming the unported pass. Task 8 ports the last two
    // passes and deletes that gate, so create() now fails on this path only
    // for a REAL reason (no device, a device that cannot preserve fp32
    // denormals, an allocation failure). Either way the message reaches the
    // user verbatim through the throw below rather than being swallowed into
    // a generic failure, which is what this comment was always about.
    spade::Result<spade::Simulation> created =
        spade::Simulation::create(scene.worlds, kStepDtNs, kSubsteps, backend);
    if (!created) {
        throw std::runtime_error("spade_viewer: Simulation::create failed for scene '" + scene.name +
                                  "': " + created.error().context);
    }
    sim.emplace(std::move(*created));
    world_count = sim->layout().world_count;

    for (const BodyPlacement& body : scene.bodies) {
        const spade::Result<spade::BodyRef> ref = sim->spawn(body.world_index, body.spawn);
        if (!ref) {
            throw std::runtime_error("spade_viewer: spawn failed for scene '" + scene.name +
                                      "': " + ref.error().context);
        }
    }

    // Vehicles (Task 20): register every model BEFORE spawning any instance
    // of it (Simulation::spawn(world, ModelTypeId, ...) needs a live id), then
    // spawn every VehiclePlacement -- both steps still BEFORE the one
    // flush_structural() call below. This is the SAME "spawn everything, then
    // flush once, then let the mesh-building code below count bodies" order
    // the plain-body loop above already uses; vehicle spawns join it rather
    // than getting a second phase or a second flush. That ordering is exactly
    // what keeps this constructor immune to the classic garbage-instance bug:
    // v1's mesh instance COUNT is fixed once, below, from world_bodies() --
    // if any body or vehicle existed only in the arena's RESERVED (not yet
    // initialized) state when that count was taken, or if a spawn landed
    // after it, the fixed-size instance buffer and the live body set would
    // permanently disagree, which reads on screen as stray/degenerate
    // instances at stale or zero-filled transforms (garbage geometry). Every
    // reservation AND its flush happen here, in this one block, before a
    // single mesh is built.
    std::vector<spade::ModelTypeId> model_ids;
    model_ids.reserve(scene.models.size());
    for (const spade::vehicles::ModelType& model : scene.models) {
        const spade::Result<spade::ModelTypeId> id = sim->register_model(model);
        if (!id) {
            throw std::runtime_error("spade_viewer: register_model failed for scene '" + scene.name +
                                      "': " + id.error().context);
        }
        model_ids.push_back(*id);
    }
    vehicle_refs.reserve(scene.vehicles.size());
    for (const VehiclePlacement& v : scene.vehicles) {
        if (v.model_index >= model_ids.size()) {
            throw std::runtime_error("spade_viewer: scene '" + scene.name +
                                      "' vehicle names model_index " + std::to_string(v.model_index) +
                                      " but only " + std::to_string(model_ids.size()) +
                                      " model(s) were registered");
        }
        const spade::Result<spade::VehicleRef> ref =
            sim->spawn(v.world_index, model_ids[v.model_index], v.spawn);
        if (!ref) {
            throw std::runtime_error("spade_viewer: vehicle spawn failed for scene '" + scene.name +
                                      "': " + ref.error().context);
        }
        vehicle_refs.push_back(*ref);
    }

    // Spawns are QUEUED until the next step boundary (sim/simulation.hpp) --
    // flush now so world_bodies() returns real poses for the very first
    // frame instead of a step's worth of zero-filled reserved slots.
    if (const spade::Result<void> flushed = sim->flush_structural(); !flushed) {
        throw std::runtime_error("spade_viewer: flush_structural failed for scene '" + scene.name +
                                  "': " + flushed.error().context);
    }

    // --- v1: window, camera -------------------------------------------------
    engine.SetupEngineWindow(1280, 720, "Spade Viewer - " + scene.name);

    const Spade::EntityID camera_id = universe.CreateEntityID();
    Spade::Entity camera(camera_id, &universe);
    camera.AddComponent<Spade::TransformComponent>();
    camera.AddComponent<Spade::CameraComponent>();
    camera.AddComponent<Spade::InputComponent>();

    camera_transform = camera.GetComponent<Spade::TransformComponent>();
    camera_transform->transform.position = scene.camera_position;
    const glm::vec3 forward = glm::normalize(scene.camera_target - scene.camera_position);
    camera_transform->transform.rotation = glm::quatLookAt(forward, glm::vec3(0.0f, 1.0f, 0.0f));

    Spade::CameraComponent* camera_component = camera.GetComponent<Spade::CameraComponent>();
    camera_component->fov = 60.0f;
    camera_component->nearPlane = 0.05f;
    camera_component->farPlane = 500.0f;
    camera_component->isActive = true;

    Spade::InputComponent* input = camera.GetComponent<Spade::InputComponent>();
    input->speed = 6.0f;
    input->bindings[GLFW_KEY_W] = MoveForward;
    input->bindings[GLFW_KEY_S] = MoveBackward;
    input->bindings[GLFW_KEY_A] = MoveLeft;
    input->bindings[GLFW_KEY_D] = MoveRight;
    input->bindings[GLFW_KEY_SPACE] = MoveUp;
    input->bindings[GLFW_KEY_LEFT_SHIFT] = MoveDown;

    // --- v1: mesh entities ---------------------------------------------------
    // ALL THREE MeshComponent insertions happen before ANY pointer into that
    // pool is read. ComponentPool<T>::m_Data (v1, Spade/Core/Objects.hpp) is a
    // plain std::vector<T>; a later AddComponent<MeshComponent>() can reallocate it
    // and invalidate a pointer obtained from an earlier one -- there are
    // three MeshComponent-bearing entities (static boxes, static spheres,
    // dynamic bodies), all added here, none ever added again afterward, so
    // pointers fetched once below are stable for the Viewer's whole lifetime.
    Spade::Entity static_boxes(universe.CreateEntityID(), &universe);
    static_boxes.AddComponent<Spade::MeshComponent>();
    Spade::Entity static_spheres(universe.CreateEntityID(), &universe);
    static_spheres.AddComponent<Spade::MeshComponent>();
    Spade::Entity dynamic_bodies(universe.CreateEntityID(), &universe);
    dynamic_bodies.AddComponent<Spade::MeshComponent>();

    Spade::MeshComponent* box_mesh = static_boxes.GetComponent<Spade::MeshComponent>();
    Spade::MeshComponent* sphere_mesh = static_spheres.GetComponent<Spade::MeshComponent>();
    dynamic_mesh = dynamic_bodies.GetComponent<Spade::MeshComponent>();

    box_mesh->mesh = Spade::GenerateCube(1.0f);
    sphere_mesh->mesh = Spade::GenerateSphere(1.0f, 24, 16);
    dynamic_mesh->mesh = Spade::GenerateSphere(1.0f, 18, 12);

    for (const spade::WorldInstanceDesc& instance : scene.worlds.worlds) {
        add_static_geometry(instance.world.sdf, *box_mesh, *sphere_mesh, scene.ground_color,
                             scene.ground_half_extent, scene.ground_thickness);
    }

    // Dynamic bodies: one sphere instance per spawned body, walked in
    // world_bodies() order -- Scene's doc comment is the invariant (capacity
    // == spawn count per world) that makes this a fixed 1:1 correspondence,
    // matching step_and_sync()'s per-frame walk exactly.
    for (uint32_t w = 0; w < world_count; ++w) {
        const spade::Result<std::span<const spade::BodyState>> bodies_span = sim->world_bodies(w);
        if (!bodies_span) {
            throw std::runtime_error("spade_viewer: world_bodies failed: " + bodies_span.error().context);
        }
        glm::vec4 color(1.0f);
        bool found_color = false;
        for (const BodyPlacement& body : scene.bodies) {
            if (body.world_index == w) {
                color = body.color;
                found_color = true;
                break;
            }
        }
        // Vehicles (Task 20): a world with no BodyPlacement (every
        // hover/wind/flight/swarm lane) falls through to its
        // VehiclePlacement's color instead -- same one-color-per-world
        // convention, just a second source for it.
        if (!found_color) {
            for (const VehiclePlacement& v : scene.vehicles) {
                if (v.world_index == w) {
                    color = v.color;
                    break;
                }
            }
        }
        // PER-BODY, not a blanket per-world constant (S6 hygiene: T2-era
        // review finding, "visual should match physics" -- the checkpoint
        // demo scene's own draw radius must track whatever the PHYSICS pass
        // actually used for this body, not a world-level default that a
        // per-body override (S6 Task 2's BodyState::proxy_radius, sentinel-0
        // == "use the world default") can leave stale for exactly this
        // body). physics::effective_proxy_radius() is the ONE canonical
        // accessor every contact kernel (CPU and GPU) reads through; using
        // it here rather than re-deriving the sentinel rule is what keeps
        // the viewer from becoming a third, driftable spelling of it.
        const float world_default_radius = scene.worlds.worlds[w].contacts.proxy_radius;
        for (const spade::BodyState& b : *bodies_span) {
            const float radius = spade::physics::effective_proxy_radius(b, world_default_radius);
            push_sphere(*dynamic_mesh, b.pos, radius, color);
        }
    }

    // Build ONCE (brief: "build all MeshComponent instances ONCE at scene
    // setup"): uploads every VAO/VBO/EBO and the initial instance SSBOs.
    // Every subsequent LoadInstanceBuffers call (step_and_sync()) hits the
    // SubData/update branch because the buffers already exist and every
    // instance count above is now fixed for good.
    engine.LoadInstanceBuffers(universe);
    engine.LoadCameraBuffers(universe);
}

void Viewer::Impl::step_and_sync() {
    accumulator_s += engine.GetDeltaTime();
    if (accumulator_s > kMaxAccumulatedDebtSeconds) {
        accumulator_s = kMaxAccumulatedDebtSeconds;
    }

    bool stepped_any = false;
    while (accumulator_s >= kStepDtSeconds) {
        // The flight scene's scripted command profile (Task 20): runs BEFORE
        // the step it commands, handed the tick about to execute -- see
        // bridge.hpp's VehicleCommandHook doc comment for why this mirrors
        // engine/testing/replay.hpp's Scenario::input exactly. A no-op for
        // every scene that leaves the hook unset (hover/wind/swarm/every
        // pre-Task-20 scene).
        if (scene.command_hook != nullptr) {
            scene.command_hook(*sim, sim->tick().value, vehicle_refs);
        }
        const spade::Result<void> stepped = sim->step(1);
        if (!stepped) {
            throw std::runtime_error("spade_viewer: Simulation::step failed: " + stepped.error().context);
        }
        accumulator_s -= kStepDtSeconds;
        stepped_any = true;
    }
    if (!stepped_any) {
        return;  // no whole physics step elapsed this frame; nothing to sync
    }

    // Copy poses into the SAME instance slots built in the constructor, in
    // the SAME world_bodies() order: counts and slot order never change,
    // only the contents. This is v1's LoadInstanceBuffers SubData contract
    // (Engine.cpp: "counts never change after init").
    uint32_t instance_i = 0;
    for (uint32_t w = 0; w < world_count; ++w) {
        const spade::Result<std::span<const spade::BodyState>> bodies_span = sim->world_bodies(w);
        if (!bodies_span) {
            throw std::runtime_error("spade_viewer: world_bodies failed: " + bodies_span.error().context);
        }
        for (const spade::BodyState& b : *bodies_span) {
            Spade::Transform& transform = dynamic_mesh->instanceTransforms[instance_i];
            transform.position = b.pos;
            // v1's Transform::rotation and v2's BodyState::orient are both
            // glm::quat, from the SAME vendored glm (1.0.1, no
            // GLM_FORCE_QUAT_DATA_WXYZ): constructor order (w,x,y,z), MEMORY
            // order (x,y,z,w) -- state/layout.hpp's documented GLM caveat.
            // Vertex.vert's quatToMat4(vec4 rotation) reads instanceTransforms
            // straight off that SSBO and consumes it as (q.x,q.y,q.z,q.w),
            // i.e. the raw memory order -- so a body->world quaternion copies
            // in directly, with no axis swizzle and no w-first/w-last
            // reordering, in EITHER direction of this pipeline.
            transform.rotation = b.orient;
            ++instance_i;
        }
    }

    // SubData branch (Engine.cpp): the buffers already exist (constructor's
    // initial call), so this is glBufferSubData, not a reallocation --
    // exactly the update path the brief specifies, called on the same fixed
    // instance counts every frame.
    engine.LoadInstanceBuffers(universe);
}

void Viewer::Impl::run() {
    while (engine.IsRunning()) {
        engine.ProcessInput(universe);

        // --- Camera orbit (Task 20; bridge.hpp's Scene::camera_orbit doc
        // comment has the full rationale) -----------------------------------
        // Recomputed and re-uploaded EVERY frame, deliberately after
        // ProcessInput(): v1's own free-fly bindings (WASD) mutate this same
        // TransformComponent through ProcessInput() and push their own
        // Engine::LoadCameraBuffers() call when a key is down (Engine.cpp) --
        // so an orbiting scene's camera wins that race every frame, which is
        // the intended "show itself off" behaviour for a demo scene.
        // Engine::LoadCameraBuffers() is v1's own PUBLIC re-upload entry
        // point (used identically at setup, above, and internally by
        // ProcessInput()); calling it again here is ordinary use of that
        // public API, not a v1 edit.
        if (scene.camera_orbit) {
            camera_orbit_elapsed_s += engine.GetDeltaTime();
            const float angle = scene.camera_orbit_angular_rate * camera_orbit_elapsed_s;
            const glm::vec3 position =
                scene.camera_orbit_target +
                glm::vec3(scene.camera_orbit_radius * std::cos(angle), scene.camera_orbit_height,
                          scene.camera_orbit_radius * std::sin(angle));
            camera_transform->transform.position = position;
            const glm::vec3 forward = glm::normalize(scene.camera_orbit_target - position);
            camera_transform->transform.rotation = glm::quatLookAt(forward, glm::vec3(0.0f, 1.0f, 0.0f));
            engine.LoadCameraBuffers(universe);
        }

        step_and_sync();
        engine.RenderColor();
        engine.DrawScene(universe, scene.clear_color);

        // --- FPS counter (user amendment A1) --------------------------------
        // Minimum bar: a rate-limited (~1 Hz, not per-frame spam) console
        // line. Reads v1's own Engine::GetFPS() (frame-pacing already uses
        // engine.GetDeltaTime(), the same wall-clock source, in
        // step_and_sync() above -- both are the tools/ wall-clock exemption,
        // global-constraints.md) rather than computing an independent FPS,
        // so there is exactly one frame-rate notion in this loop, not two
        // that could disagree.
        //
        // `fps_print_timer_s` is deliberately its OWN accumulator, separate
        // from `accumulator_s`: that one is drained by whole kStepDtSeconds
        // steps and must not carry a print-cadence remainder, and this one
        // must not perturb the physics step count -- printing is pure
        // presentation, never coupled to the fixed-step schedule.
        //
        // A window-title update (the brief's stretch goal, "if reachable
        // through v1's PUBLIC headers") is NOT reachable: Engine::
        // SetupEngineWindow sets the title once and Engine.hpp exposes no
        // public re-title call or GLFWwindow* getter -- m_GLFWwindow is
        // private, and v1 is frozen, so there is no adding one. Console-only
        // is what's left, per the brief's own instruction not to modify v1
        // to get the title.
        fps_print_timer_s += engine.GetDeltaTime();
        if (fps_print_timer_s >= 1.0f) {
            fps_print_timer_s = 0.0f;
            // THE BACKEND IS NAMED ON EVERY LINE (S6 Task 6). The FPS counter
            // itself is user amendment A1's; a headed demo whose whole claim is
            // "the Vulkan kernels are stepping this" has to STATE which path is
            // running rather than leave the user taking the command line's word
            // for it.
            std::printf("fps: %.0f | bodies: %zu | backend: %s\n", engine.GetFPS(),
                        scene.bodies.size() + scene.vehicles.size(),
                        backend.kind == spade::compute::BackendKind::vulkan ? "vulkan" : "cpu");
            // Explicit flush: stdout is fully (not line-) buffered once it is
            // NOT an interactive console -- e.g. redirected to a file/pipe by
            // a caller capturing this diagnostic line -- so without this a
            // print can sit unseen in the C runtime's buffer until the
            // process exits. At ~1 Hz the flush cost is irrelevant.
            std::fflush(stdout);
        }
    }
}

// ---------------------------------------------------------------------------
// Viewer
// ---------------------------------------------------------------------------

Viewer::Viewer(Scene scene, spade::compute::BackendDesc backend)
    : impl_(std::make_unique<Impl>(std::move(scene), backend)) {}
Viewer::~Viewer() = default;
Viewer::Viewer(Viewer&&) noexcept = default;
Viewer& Viewer::operator=(Viewer&&) noexcept = default;

void Viewer::run() { impl_->run(); }

}  // namespace spade::viewer
