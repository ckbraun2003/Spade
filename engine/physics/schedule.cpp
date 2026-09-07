#include "physics/schedule.hpp"

#include <array>

#include "objects/behavior.hpp"  // BehaviorRegistry's complete definition --
                                  // schedule.hpp only forward-declares it, and
                                  // this is the one TU that calls run_slot().
#include "physics/integrator.hpp"
#include "vehicles/rotor.hpp"  // RotorRow's complete definition -- schedule.hpp
                                // only forward-declares it (S5 T9 seam ticket);
                                // this TU is the one that walks `rotors` (below,
                                // via vehicles::apply_rotors()), so it needs the
                                // real type.

// ---------------------------------------------------------------------------
// The schedule, as data. See schedule.hpp for the §3 quote this file is a
// transcription of, and for what each pass is and is not allowed to do.
// ---------------------------------------------------------------------------

namespace spade::physics {

void pass_medium_update(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        // Unconditional -- see the "INCLUDING AT TURBULENCE LEVEL none" note in
        // the header. Five gaussians per world per substep, always, so stream
        // position never depends on configuration.
        dryden_advance(*w.dryden, *w.dryden_params, ctx.h);
    }
}

void pass_force_elements(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        // Constructed per world per substep rather than cached: two pointer
        // stores, and structurally impossible to leave dangling across a
        // snapshot restore. The view must be paired with ITS OWN world's
        // WorldParams row -- the arena row `w.params` points at -- because
        // sample() reads density and mean wind from the row it is handed while
        // the gust comes from the bound filter state. Crossing the two would be
        // silent: one world's air with another world's gusts.
        const DrydenMedium medium(*w.dryden, *w.dryden_params);
        // §3's declared order INSIDE this pass: "rotors -> drag -> lift
        // surfaces". Both calls accumulate into the same two float
        // accumulators, so the order is a numerical contract and not a
        // preference -- see the header.
        vehicles::apply_rotors(w.bodies, w.rotors, *w.sdf, medium, *w.params, ctx.h);
        apply_drag(w.bodies, w.drag_elements, medium, *w.params, ctx.h);
    }
}

void pass_gravity(const SubstepContext&) noexcept {
    // INTENTIONALLY EMPTY. Gravity application lives inside Integrate (Task 9
    // op-order contract); this pass slot is retained for schedule-shape parity
    // with spec §3 and MUST NOT also accumulate m*g. See the header.
}

void pass_collision_static(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        resolve_static_contacts(w.bodies, *w.sdf, w.contacts, ctx.h);
    }
}

void pass_collision_dynamic(const SubstepContext& ctx) noexcept {
    if (ctx.batch_dynamic_collision) {
        // D8's shape: one sweep, every world, cross-world pairs structurally
        // impossible because the sort key leads with the world id.
        resolve_dynamic_contacts(ctx.all_bodies, ctx.all_slot_to_world, ctx.dynamic_grid,
                                 ctx.dynamic_contacts, *ctx.scratch);
        return;
    }

    // Heterogeneous materials: one sweep per world, each with its own record.
    // Numerically identical per world to the batched form (see the header).
    for (const WorldSubstepView& w : ctx.worlds) {
        resolve_dynamic_contacts(w.bodies, w.body_slot_to_world, w.grid, w.contacts, *ctx.scratch);
    }
}

void pass_integrate(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        integrate_bodies(w.bodies, *w.params, ctx.h);
    }
}

void pass_sensor_synthesis(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        // Every sensor's rate divider advances by one substep here; only those
        // that reach a boundary emit. `bodies` narrows to a const span: this
        // pass READS body state (specific_force, omega_body, flags) and writes
        // only sensor rows and rings, which is what keeps it safely after
        // Integrate rather than interleaved with it.
        sensors::synthesize_imu(w.bodies, w.imu_sensors, w.imu_ring, ctx.tick.value);
    }
}

void pass_publish(const SubstepContext&) noexcept {
    // No-op stub. The tick increment lives in Simulation::step() (Tick counts
    // steps, not substeps); the snapshot ring and the frame-state copy point
    // are S6. See the header for why the slot exists anyway.
}

// BehaviorsKinematic -- runs the registry's kinematic slot, or NOTHING when no
// registry is attached. The slot's position is the ruling; see schedule.hpp
// for why it sits between MediumUpdate and ForceElements.
//
// THE NULL CHECK IS WHAT KEEPS TASK 7'S GOLDEN PROOF ALIVE. Every scenario in
// the corpus attaches no registry, so both of these remain the empty passes
// that were proved byte-identical -- and the golden corpus keeps proving it on
// every run, rather than that proof having been a one-time observation.
void pass_behaviors_kinematic(const SubstepContext& ctx) noexcept {
    if (ctx.behaviors != nullptr) {
        ctx.behaviors->run_slot(objects::BehaviorSlot::kinematic, ctx);
    }
}

// BehaviorsForce -- after ForceElements, so behavior wrenches never perturb the
// pinned rotors-then-drag float accumulation. See schedule.hpp.
void pass_behaviors_force(const SubstepContext& ctx) noexcept {
    if (ctx.behaviors != nullptr) {
        ctx.behaviors->run_slot(objects::BehaviorSlot::force, ctx);
    }
}

namespace {

// The pass list. `constexpr` and file-scope: there is no code path that can
// mutate it, and substep_schedule() hands out a span over exactly this object.
//
// The names are the spec's names, spelled exactly as §3 spells them -- they are
// what test_determinism.cpp compares against, and what a future profiler or
// debug HUD labels a timing row with. The two Behaviors* names are SL6's, in
// the positions the user ruled.
constexpr std::array<Pass, kSubstepPassCount> kSchedule{{
    {"MediumUpdate", &pass_medium_update},
    {"BehaviorsKinematic", &pass_behaviors_kinematic},
    {"ForceElements", &pass_force_elements},
    {"BehaviorsForce", &pass_behaviors_force},
    {"Gravity", &pass_gravity},
    {"CollisionStatic", &pass_collision_static},
    {"CollisionDynamic", &pass_collision_dynamic},
    {"Integrate", &pass_integrate},
    {"SensorSynthesis", &pass_sensor_synthesis},
    {"Publish", &pass_publish},
}};

}  // namespace

std::span<const Pass> substep_schedule() noexcept { return std::span<const Pass>(kSchedule); }

void run_substep(const SubstepContext& ctx) noexcept {
    // Deliberately driven BY the schedule array rather than by eight hand-written
    // calls: this is the only substep driver in the engine, so if it did not read
    // the declared order, the declared order would be documentation instead of a
    // contract.
    for (const Pass& pass : kSchedule) {
        pass.run(ctx);
    }
}

}  // namespace spade::physics
