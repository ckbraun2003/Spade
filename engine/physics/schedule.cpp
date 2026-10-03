#include "physics/schedule.hpp"

#include "objects/behavior.hpp"  // BehaviorRegistry's complete definition --
                                  // schedule.hpp only forward-declares it, and
                                  // this is the one TU that calls run_slot().
#include "physics/integrator.hpp"
#include "physics/sampled_medium.hpp"

// ---------------------------------------------------------------------------
// The passes, as functions. The order they run in is the compiled module
// schedule's (sim/module.hpp, sim/standard_modules.cpp); schedule.hpp says what
// each pass is and is not allowed to do.
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

// The ForceElements chain, split into its two modules' passes. Each world's
// bodies still receive rotors' wrench before drag's: the loops run world by
// world, and worlds share no body, so splitting the per-world call pair into
// two all-world loops changes no accumulation order.
void pass_environment_sample(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        const WorldParams& p = *w.params;
        w.fields[kFieldGravityOffset + 0] = p.gravity.x;
        w.fields[kFieldGravityOffset + 1] = p.gravity.y;
        w.fields[kFieldGravityOffset + 2] = p.gravity.z;
        w.fields[kFieldDensityOffset] = p.air_density;
    }
}

void pass_dryden_sample(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        // DrydenMedium::sample()'s expression, verbatim (world/medium.cpp).
        const glm::vec3 wind = w.params->wind + dryden_turbulence(*w.dryden, *w.dryden_params);
        w.fields[kFieldWindOffset + 0] = wind.x;
        w.fields[kFieldWindOffset + 1] = wind.y;
        w.fields[kFieldWindOffset + 2] = wind.z;
    }
}

// Rotors and drag read the medium the providers stored this substep. The model
// functions take a Medium, so they are unchanged; the stored values are the
// bits DrydenMedium would have computed inline.
void pass_rotor_forces(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        const SampledMedium medium(w.fields);
        vehicles::apply_rotors(w.bodies, w.rotors, *w.sdf, medium, *w.params, ctx.h);
    }
}

void pass_drag(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        const SampledMedium medium(w.fields);
        apply_drag(w.bodies, w.drag_elements, medium, *w.params, ctx.h);
    }
}

// SensorSynthesis, split the same way. IMU and GNSS read bodies as const and
// write disjoint arrays from separately tagged streams, so they commute.
void pass_sensor_imu(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        sensors::synthesize_imu(w.bodies, w.imu_sensors, w.imu_ring, ctx.tick.value);
    }
}

void pass_sensor_gnss(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        sensors::synthesize_gnss(w.bodies, w.gnss_sensors, w.gnss_ring, ctx.tick.value);
    }
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
        // Gravity is a field (engine A9: Integrate applies it, nothing else
        // does), copied from WorldParams by environment.sample this substep.
        const glm::vec3 gravity(w.fields[kFieldGravityOffset + 0], w.fields[kFieldGravityOffset + 1],
                                w.fields[kFieldGravityOffset + 2]);
        integrate_bodies(w.bodies, gravity, ctx.h);
    }
}

void pass_behaviors_kinematic(const SubstepContext& ctx) noexcept {
    if (ctx.behaviors != nullptr) {
        ctx.behaviors->run_slot(objects::BehaviorSlot::kinematic, ctx);
    }
}

// behaviors.force -- last in Forces, so behavior wrenches never perturb the
// pinned rotors-then-drag float accumulation. See schedule.hpp.
void pass_behaviors_force(const SubstepContext& ctx) noexcept {
    if (ctx.behaviors != nullptr) {
        ctx.behaviors->run_slot(objects::BehaviorSlot::force, ctx);
    }
}

}  // namespace spade::physics
