// ---------------------------------------------------------------------------
// medium_update.slang (S6 Task 8) -- the dryden.advance pass, ported from
// engine/physics/schedule.cpp's pass_medium_update(). A Fields pass, and in the
// standard set the first dispatch of every substep.
//
// The whole pass is one line on the CPU:
//
//     for (const WorldSubstepView& w : ctx.worlds)
//         dryden_advance(*w.dryden, *w.dryden_params, ctx.h);
//
// and one line here. Everything it does lives in engine/shaders/dryden.slang
// (the port of world/medium.cpp) and engine/shaders/rng.slang (the port of
// core/rng.hpp); this file is the DISPATCH SHAPE and nothing else.
//
// ===========================================================================
// ONE THREAD PER WORLD -- THE ONE STRUCTURAL DECISION IN THIS FILE, AND IT IS
// FORCED, NOT CHOSEN.
//
// The Dryden filter is a SEQUENTIAL RECURRENCE within a world: this substep's
// five normalized states are a function of the previous substep's five and of
// five fresh gaussians drawn from the world's own rng::Stream, in a pinned
// order, with the stream's 16 bytes carried forward. There is nothing to
// parallelize inside one world -- the very next substep needs the state this
// one produced, and the very next draw needs the stream position this one left.
//
// WORLDS ARE THE PARALLEL AXIS, and they are genuinely independent: each world
// has its own DrydenState row (direct-indexed by world id, capacity 1 per
// world) and its own stream, derived from its own seed under the "dryden"
// domain tag, so no two worlds' filters can observe each other. Running them
// concurrently is therefore the same computation as the CPU's world loop with
// independent parts overlapped -- not an approximation of it.
//
// THIS IS THE SAME POSTURE collision_dynamic.slang TAKES for the Gauss-Seidel
// sweep, and for the same reason: the parity contract names an order, so the
// port reproduces the order and parallelizes only where the CPU itself is
// decomposable. It is stated here too rather than cross-referenced, because a
// reader arriving at a one-thread-per-world dispatch grid deserves to find the
// argument in the file that uses it.
//
// NO WORKGROUP-SHARED ANYTHING, and therefore no workgroup_size dependence
// (T9's A7 obligation): a world's whole filter update happens in one lane,
// with no barrier(), no groupshared array and no cross-lane reduction.
//
// ===========================================================================
// WHY THE STREAM IS LOADED, ADVANCED AND STORED AS A WHOLE ROW
//
// dryden_advance() takes the state `inout` and rng.slang draws through it, so
// the row is read once into a local, mutated five draws and three channel
// updates later, and written back once. A read-modify-write through the buffer
// per draw would be the same arithmetic with ten times the traffic and one more
// thing to reason about; and since this thread OWNS world `w`'s row outright
// (one thread per world, and nothing else in the substep writes `dryden`),
// there is no interleaving for the single store to lose.
//
// `_p0` rides through unchanged: it is read with the row and written back with
// it, which is exactly what the CPU's dryden_advance() leaves it as -- it never
// touches that lane either. (dryden_init(), which DOES zero it, runs host-side
// at world construction and never on this path.)
//
// ===========================================================================
// DIVISION AND SQRT SITES: none in this file. dryden.slang and rng.slang list
// their own; between them they are why the gust is banded rather than
// bit-identical, and rng.slang's header names the single operation responsible
// (Box-Muller's sqrt).
// ---------------------------------------------------------------------------

// `import layouts` AS WELL AS `import bindings`: Slang's `import` is NOT
// transitive, and this kernel names element types (DrydenState, DrydenParams).
import layouts;
import bindings;
import dryden;

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
void medium_update(uint3 tid : SV_DispatchThreadID) {
    const uint world = tid.x;
    if (world >= pass_params.world_count) {
        return;
    }

    // `dryden` and `dryden_params` are BOTH direct-indexed by world id --
    // capacity 1 per world, so global slot w IS world w. That is why
    // bindings.slang deliberately does NOT bind dryden.slot_to_world (its
    // section B: "capacity 1 per world; the map is the identity and a kernel
    // already has the world index as its dispatch coordinate").
    DrydenState state = dryden[world];
    const DrydenParams params = dryden_params[world];

    // physics/schedule.cpp:24 -- UNCONDITIONAL, including at turbulence level
    // `none`. world/medium.hpp's determinism note is why: five gaussians are
    // drawn every substep whatever the sigmas are, so the stream position is a
    // function of the substep count alone. A kernel that skipped the draws for
    // a calm world would put that world's stream at a different position from
    // the CPU's on the very first substep, and every LATER scenario that turned
    // turbulence on would then disagree.
    dryden_advance(state, params, pass_params.h);

    dryden[world] = state;
}
