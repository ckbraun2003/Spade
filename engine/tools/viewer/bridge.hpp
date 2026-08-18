#pragma once

// ---------------------------------------------------------------------------
// bridge.hpp -- the strangler viewer's PUBLIC surface (Task 14, engine design
// spec S3 "the strangler viewer": v2 physics feeds v1's frozen OpenGL
// renderer).
//
// DELIBERATELY V1-FREE. Only glm, v2 (spade::) and std types appear below, so
// scenes.cpp (which builds Scene values) and main.cpp (which drives the
// Viewer) never see a v1 header (Spade/Spade.hpp and everything it pulls in
// -- glad, GLFW, windows.h). bridge.cpp is the ONE translation unit that
// includes both v1 and v2 headers; see its file comment for why that
// isolation exists and what it costs.
//
// The Viewer type below is an opaque pImpl for exactly that reason: every v1
// type (Spade::Engine, Spade::Universe, Spade::MeshComponent, ...) is a
// private implementation detail of bridge.cpp, never named here.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "compute/backend.hpp"  // BackendDesc (S6 Task 6) -- Vulkan-free by design
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"

namespace spade::viewer {

// ---------------------------------------------------------------------------
// One dynamic body to spawn at scene setup, plus which world it belongs to
// and how to render it.
//
// `world_index` indexes `Scene::worlds.worlds`. Scenes MUST list bodies
// grouped in ascending world_index order (all of world 0's bodies, then all
// of world 1's, ...) -- that is the render-order contract bridge.cpp's
// per-frame update relies on; see its file comment for the full argument.
// ---------------------------------------------------------------------------
struct BodyPlacement {
    uint32_t world_index = 0;
    spade::BodySpawn spawn;
    glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

// ---------------------------------------------------------------------------
// One VEHICLE to spawn at scene setup (Task 20): which registered model
// (`Scene::models`, by index -- MODELS ARE REGISTERED IN `Scene::models`
// ORDER, so `model_index` is that order, not a ModelTypeId), which world, its
// initial pose/velocity/rotor speed, and how to render it.
//
// A vehicle spawn still lands in the SAME `bodies` arena a BodyPlacement does
// (Simulation::spawn(world, model, where) composes one body plus its rotor,
// drag and sensor rows -- sim/simulation.hpp), so bridge.cpp's existing
// world_bodies()-driven mesh walk already renders it -- as a plain sphere,
// the same rendering-only approximation every other dynamic body gets (v1's
// mesh set has no rotor-arm geometry and v1 is frozen; see bridge.cpp's file
// comment). Scene::bodies' capacity invariant below applies identically to
// vehicles: a world's declared Capacities::bodies must count every
// BodyPlacement AND every VehiclePlacement that names it.
// ---------------------------------------------------------------------------
struct VehiclePlacement {
    uint32_t world_index = 0;
    uint32_t model_index = 0;  // index into Scene::models
    spade::VehicleSpawn spawn;
    glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
};

// ---------------------------------------------------------------------------
// The flight scene's scripted per-tick command hook (Task 20).
//
// A BARE FUNCTION POINTER, the same shape physics/schedule.hpp's PassFn is
// and for the same reason: a scene is data, and a bare function cannot
// accidentally close over mutable state or a clock, which is what keeps a
// command profile a PURE function of (tick, vehicle index) -- deterministic,
// tick-driven, replayable -- rather than the general scripting system the
// brief explicitly rules out (YAGNI: one hook slot, no expression language,
// no per-scene state machine framework).
//
// Called once per STEP, BEFORE that step runs, with the tick ABOUT TO BE
// EXECUTED -- the identical contract engine/testing/replay.hpp's
// Scenario::input uses ("input runs once per step, BEFORE that step, and is
// handed the tick that is about to be executed"), so a scene's command
// profile reads exactly like the determinism corpus' own input scripts.
// `vehicles` is every VehicleRef bridge.cpp spawned from `Scene::vehicles`,
// in that same order, so `vehicles[i]` is `Scene::vehicles[i]`'s live ref.
//
// A hook that hits an unexpected engine error (e.g. a stale ref) throws --
// scenes.cpp's own house style for "this must not fail" (see
// unwrap_world()'s comment); bridge.cpp does not catch on its behalf.
// ---------------------------------------------------------------------------
using VehicleCommandHook = void (*)(spade::Simulation& sim, uint64_t tick,
                                     std::span<const spade::VehicleRef> vehicles);

// ---------------------------------------------------------------------------
// Everything one demo needs: the v2 world set, the bodies to spawn into it,
// and the v1 camera/render presentation. A Scene is pure data -- building one
// touches no engine or GL state (see scenes.cpp).
//
// EACH WORLD IN `worlds.worlds` MUST DECLARE Capacities::bodies EQUAL TO THE
// NUMBER OF ENTRIES IN `bodies` PLUS `vehicles` THAT NAME IT, exactly.
// Simulation::world_bodies() returns a world's WHOLE arena partition
// (WorldSetLayout::body_capacity slots, the set's max, not a "how many are
// live" count) -- so if capacity ever exceeds the spawn count, the render
// loop would see extra always-inactive slots at the origin. Scenes are built
// so capacity == spawn count identically in every world, which is what lets
// the dynamic-body mesh be sized once, at setup, from the world set alone
// (scenes.cpp keeps this invariant by construction, one WorldBuilder per
// lane/world with its own tight capacity). `Capacities::force_elements` and
// `::sensors` carry the analogous invariant for a vehicle's rotor/drag rows
// and IMU mount(s) -- see vehicles/model_type.hpp's ModelType::
// force_element_count().
// ---------------------------------------------------------------------------
struct Scene {
    std::string name;
    spade::WorldSetDesc worlds;
    std::vector<BodyPlacement> bodies;

    // Vehicles (Task 20). `models` is registered, IN ORDER, before any spawn
    // -- register_model() mints one-based ids in registration order
    // (sim/simulation.hpp), so `models[i]` becomes ModelTypeId{i + 1} and
    // VehiclePlacement::model_index indexes THIS vector, not a raw id.
    // `vehicles` is spawned (and, if `command_hook` is set, subsequently
    // commanded) after every model is registered.
    std::vector<spade::vehicles::ModelType> models;
    std::vector<VehiclePlacement> vehicles;
    VehicleCommandHook command_hook = nullptr;

    // ---------------------------------------------------------------------
    // Camera orbit (Task 20; "camera orbit" for the hover scene). A cheap,
    // v1-edit-free deterministic showcase orbit through v1's OWN public
    // camera plumbing (Engine::LoadCameraBuffers -- see bridge.cpp) rather
    // than a static camera, per the coordinator's fallback ruling: this did
    // NOT require touching v1's frozen src/include, so the orbit is used
    // rather than the static-camera fallback.
    //
    // WALL-CLOCK DRIVEN, DELIBERATELY -- the tools/ pacing exemption
    // (global-constraints.md) that A1's FPS counter already relies on
    // (bridge.cpp's `fps_print_timer_s`). Camera motion is pure presentation,
    // never read back by the physics or by a test, so there is no
    // determinism obligation here to buy by tying it to the tick instead.
    //
    // When enabled, OVERRIDES v1's free-fly input every frame (the orbit is
    // recomputed and re-uploaded after ProcessInput() -- see bridge.cpp's
    // run()): a demo scene is meant to show itself off without requiring the
    // user to fly the camera, and free-fly remains available on any scene
    // that leaves this off.
    // ---------------------------------------------------------------------
    bool camera_orbit = false;
    glm::vec3 camera_orbit_target{0.0f};    // look-at point; also the orbit's XZ center
    float camera_orbit_radius = 6.0f;       // m, horizontal distance from the target
    float camera_orbit_height = 2.0f;       // m, camera height ABOVE the target
    float camera_orbit_angular_rate = 0.3f; // rad/s

    // Rendering-only approximation knobs for the plane->box mapping (see
    // bridge.cpp's SDF->mesh notes): a `plane` SDF primitive is an infinite
    // half-space with no inherent extent, so the viewer renders it as a large
    // finite slab of this size, centered/oriented at the primitive's pose.
    float ground_half_extent = 6.0f;  // rendered slab half-width/-depth, m
    float ground_thickness = 0.4f;    // rendered slab thickness, m

    // Color for every STATIC world-geometry instance (ground slabs, boxes,
    // spheres, torus rings) -- one flat color per scene, not just the ground.
    glm::vec4 ground_color{0.55f, 0.58f, 0.62f, 1.0f};

    glm::vec3 camera_position{0.0f, 4.0f, 12.0f};
    glm::vec3 camera_target{0.0f, 0.0f, 0.0f};
    glm::vec4 clear_color{0.04f, 0.05f, 0.07f, 1.0f};
};

// The scene catalogue (scenes.cpp). `make_scene` returns nullopt for an
// unrecognized name; `scene_names` is what main.cpp lists on a bad argument.
[[nodiscard]] std::optional<Scene> make_scene(std::string_view name);
[[nodiscard]] const std::vector<std::string>& scene_names();

// ---------------------------------------------------------------------------
// Viewer -- owns the v1 GL window/Universe/Engine AND the v2 Simulation, and
// drives the render loop until the window closes.
//
// Move-only (holds v1's Engine, which owns a GLFW window and is itself
// non-movable -- see bridge.cpp's Impl), via the pImpl indirection: moving a
// Viewer only ever moves the unique_ptr, never the Impl object itself, so
// v1's non-movability never surfaces here.
// ---------------------------------------------------------------------------
class Viewer {
public:
    // `backend` (S6 Task 6): which compute backend steps this scene.
    // DEFAULTED, so every existing construction is unchanged and still runs
    // the CPU schedule. Passing BackendDesc{.kind = vulkan} routes the same
    // scene, the same spawns and the same step decomposition through the
    // ported kernels -- see bridge.cpp's Impl constructor. compute/backend.hpp
    // is Vulkan-FREE (its own standing note), so naming the type here does not
    // put a Vulkan header on this tool's include path.
    explicit Viewer(Scene scene, spade::compute::BackendDesc backend = {});
    ~Viewer();
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;
    Viewer(Viewer&&) noexcept;
    Viewer& operator=(Viewer&&) noexcept;

    // Opens the window, builds every v1 MeshComponent ONCE, then steps the
    // v2 Simulation and renders until the window closes (Esc or the close
    // button -- v1's own ProcessInput bindings, untouched).
    void run();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::viewer
