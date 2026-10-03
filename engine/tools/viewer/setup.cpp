// ---------------------------------------------------------------------------
// setup.cpp -- see setup.hpp. The body below moved verbatim from
// bridge.cpp's Viewer::Impl constructor; `sim` and `vehicle_refs` are
// locals here so every statement kept its exact text and order.
// ---------------------------------------------------------------------------
#include "setup.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace spade::viewer {

SceneRun make_simulation(const Scene& scene, spade::compute::BackendDesc backend) {
    std::optional<spade::Simulation> sim;
    std::vector<spade::VehicleRef> vehicle_refs;

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

    return SceneRun{std::move(*sim), std::move(vehicle_refs)};
}

}  // namespace spade::viewer
