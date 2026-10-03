// ---------------------------------------------------------------------------
// forces_drag.slang (S6 Task 6) -- the drag.forces pass (DragBody forces),
// ported from engine/physics/forces.cpp's apply_drag().
//
// ===========================================================================
// ONE THREAD PER BODY, NOT PER ELEMENT -- THE ONE STRUCTURAL DECISION IN THIS
// FILE, AND IT IS FORCED THREE TIMES OVER.
//
// The CPU pass iterates ELEMENTS and accumulates into their bodies:
//
//     for (elem : elems) { ...; body.force_acc += F_world;
//                               body.torque_acc += cross(r, F_body); }
//
// A per-element dispatch would be the literal transcription, and it would be
// wrong on all three of the axes this program cares about:
//
//   1. CORRECTNESS. Two elements on the same body would read-modify-write the
//      same two float3 accumulators from two threads -- a data race, not merely
//      an ordering question.
//   2. DETERMINISM. Even made safe with atomics, floating-point addition is not
//      associative, so the result would depend on which thread got there first.
//      The corpus digest would move run to run on the same device.
//   3. THE STANDING KERNEL-DESIGN CONSTRAINT (global constraints, carried from
//      the pre-flight scan): "no workgroup-shared reduction whose result order
//      depends on local size -- per-item writes or fixed-order serial folds
//      only on parity paths, so T9's workgroup_size {32,64,128} bit-invariance
//      holds by construction". A per-element dispatch is precisely the shape
//      that rule excludes.
//
// So each thread owns ONE BODY and performs the FIXED-ORDER SERIAL FOLD the
// rule sanctions: it walks its world's element partition in ascending slot
// order -- the CPU's own visit order -- accumulating in registers, and stores
// the two accumulators exactly once. Same operands, same order, one write.
//
// THE COST is O(bodies x elements_per_world) instead of O(elements): every body
// thread reads every element row of its own world. That is the honest price of
// a race-free, order-pinned fold, it is bounded by the world's DECLARED element
// capacity (4 in the golden corpus), and nothing about the arithmetic changes
// if a future task wants a two-phase scatter instead.
//
// A BODY WITH NO MATCHING ELEMENT IS NOT WRITTEN AT ALL (the `touched` flag
// below), so this pass leaves such a body byte-for-byte untouched exactly as
// the CPU loop does -- rather than storing back the same bits, which would be
// numerically identical but would make "this pass wrote nothing" false.
//
// ===========================================================================
// THE MEDIUM IS THE REAL ONE AS OF S6 TASK 8, AND THAT IS THIS FILE'S ONLY
// SUBSTANTIVE CHANGE IN THAT WAVE.
//
// The CPU samples the air through the Medium seam:
//     sample = medium.sample(params, body.pos)   // DrydenMedium
//            = { params.air_density, params.wind + dryden_turbulence(...) }
//
// THROUGH S6 TASK 7 THIS KERNEL READ `air_density` and `wind` STRAIGHT OUT OF
// WorldParams AND ADDED NO GUST, because the MediumUpdate pass that advances
// the Dryden filter was a stub. That was an exact mirror only for a world whose
// turbulence sigmas are all zero (dryden_turbulence() then returns exactly
// (0,0,0) and `params.wind + 0` is `params.wind` bit for bit) -- and
// sim/simulation.cpp REFUSED any other world outright, with Code::unavailable
// naming this pass, rather than step it with the gusts silently absent.
//
// Task 8 ports MediumUpdate, so the omission and the refusal both go. This
// kernel now calls dryden.slang's dryden_medium_sample() -- the SAME function
// rotors.slang calls, spelled once, so the two force kernels cannot disagree
// about what the air is doing in a substep, exactly as the CPU hands both
// passes a DrydenMedium built from the same advanced row, once per world, by
// physics/schedule.cpp's pass_rotor_forces() and pass_drag().
//
// THE ORDERING THAT MAKES IT CORRECT is the recorded chain's: dryden.advance
// (Fields) before drag.forces (Forces), with a memory barrier between every
// adjacent dispatch -- so the `dryden` row this kernel samples is the one THIS
// substep's dryden.advance advanced, which is precisely the CPU's contract
// (world/medium.hpp: "dryden_advance() must be called EXACTLY ONCE PER SUBSTEP
// for each world ... and therefore BEFORE any force element reads sample()").
//
// ===========================================================================
// GLM EXPRESSIONS are mirrored literally, as in integrate.slang; see
// mirrors.slang for why (each helper names the vendored glm source).
//
// DIVISION AND SQRT SITES IN THIS FILE: exactly one, `glm::length(v_rel)` in
// the quadratic branch (forces.cpp:82). No division anywhere in this kernel.
// dryden.slang and rng.slang list their own, and reach here through the medium
// sample.
// ---------------------------------------------------------------------------

// `import layouts` AS WELL AS `import bindings`: Slang's `import` is NOT
// transitive -- bindings.slang imports layouts.slang for its own declarations,
// but that does not re-export the STRUCT NAMES to a module importing bindings.
// A module that only SUBSCRIPTS a buffer never needs them; one that names an
// element type (`const SdfNodeRow node = ...`) does. (The example this note
// used to give was pipeline_smoke.slang, the stub kernel S6 Task 8 deleted;
// every module in the tree now names element types, so the distinction is
// stated rather than illustrated.)
import layouts;
import bindings;
// The glm expression mirrors this kernel used to declare for itself
// (glm_cross/glm_length3/glm_quat_conjugate/glm_quat_rotate). ONE definition as
// of S6 Task 8 -- see mirrors.slang's header for the T7 review finding (minor
// M3) that forced the extraction and for the per-helper glm citations.
import mirrors;
// dryden_medium_sample() -- the DrydenMedium sample this pass reads, shared
// with rotors.slang (S6 Task 8). See the header note.
import dryden;

// state/arenas.hpp's kInvalidWorld; physics/integrator.hpp's
// body_flags::active. Sentinels, not layouts -- see integrate.slang.
static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

// physics/forces.hpp's drag_mode:: values.
static const uint kDragModeQuadratic = 0u;
static const uint kDragModeComponentwise = 1u;

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE (S6 Task 9b).
// cmake/SpadeSlang.cmake's spade_slang_kernel_variants() compiles this kernel
// once per size in SPADE_SLANG_WORKGROUP_SIZES (= compute/backend.hpp's
// kSupportedWorkgroupSizes), passing -DSPADE_WG=<size>, so that
// BackendDesc::workgroup_size selects a module that was actually compiled for
// it. There is DELIBERATELY no #ifndef fallback: a missing -D must fail the
// build loudly, not silently produce a 64-wide kernel that step_recorder.cpp
// then dispatches on a grid divided by some other number.
[numthreads(SPADE_WG, 1, 1)]
void forces_drag(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }

    const uint world = bodies_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    // forces.cpp:25 -- an element whose body is not active contributes
    // nothing, so an inactive body is skipped whole here.
    if ((bodies[slot].flags & kBodyFlagActive) == 0u) {
        return;
    }

    // DragBodyRow::body_slot is WORLD-LOCAL (physics/forces.hpp): it indexes
    // the same world's `bodies` partition, not a global arena slot. Partitions
    // are contiguous and equally sized, so the world-local index of a global
    // slot is exactly this subtraction.
    const uint local_body = slot - world * pass_params.body_capacity;

    const float4 orient = bodies[slot].orient;
    const float3 vel = bodies[slot].vel;

    // forces.cpp:29-30 -- the Medium sample, gust included as of S6 Task 8.
    // HOISTED OUT OF THE ELEMENT FOLD, which is exact rather than an
    // optimization: the CPU samples per element at `body.pos`, and this is a
    // frozen-field POINT model that ignores position entirely (medium.hpp
    // section 4), so every one of those samples returns the identical value.
    // Read from this world's field samples (module-API stage 3), written in
    // Fields by field_environment and field_dryden: the same bits
    // dryden_medium_sample() would compute here.
    const float density = field_samples[world].density;
    const float3 v_rel = vel - field_samples[world].wind;

    float3 force_acc = bodies[slot].force_acc;
    float3 torque_acc = bodies[slot].torque_acc;
    bool touched = false;

    // The fixed-order serial fold: ascending element slot, the CPU's own
    // visit order.
    const uint element_base = world * pass_params.element_capacity;
    for (uint e = 0; e < pass_params.element_capacity; ++e) {
        const DragBodyRow elem = drag_bodies[element_base + e];
        if (elem.enabled == 0u) continue;              // forces.cpp:19
        if (elem.body_slot != local_body) continue;    // this body's elements only

        float3 f_world;
        float3 f_body;

        if (elem.mode == kDragModeComponentwise) {
            // forces.cpp:73-77 -- rotate v_rel into the body frame, apply
            // F_i = -c_i * |v_i| * v_i per BODY axis, rotate back out.
            // `-elem.coeffs.x * fabs(vb.x) * vb.x` associates as
            // `((-c.x) * |vb.x|) * vb.x`.
            const float3 v_rel_body = glm_quat_rotate(glm_quat_conjugate(orient), v_rel);
            f_body = float3(-elem.coeffs.x * abs(v_rel_body.x) * v_rel_body.x,
                            -elem.coeffs.y * abs(v_rel_body.y) * v_rel_body.y,
                            -elem.coeffs.z * abs(v_rel_body.z) * v_rel_body.z);
            f_world = glm_quat_rotate(orient, f_body);
        } else {
            // forces.cpp:82-84 -- F = -1/2 * rho * Cd * A * |v_rel| * v_rel,
            // WORLD frame. The scalar chain associates left, so the whole
            // coefficient is folded to one float BEFORE the vector multiply.
            const float speed = glm_length3(v_rel);
            f_world = (-0.5f * density * elem.coeffs.x * elem.area * speed) * v_rel;
            f_body = glm_quat_rotate(glm_quat_conjugate(orient), f_world);
        }

        // forces.cpp:87-88 -- world-frame force, BODY-frame torque
        // (state/layout.hpp's deliberate accumulator asymmetry).
        force_acc = force_acc + f_world;
        torque_acc = torque_acc + glm_cross(elem.local_pos, f_body);
        touched = true;
    }

    if (touched) {
        bodies[slot].force_acc = force_acc;
        bodies[slot].torque_acc = torque_acc;
    }
}
