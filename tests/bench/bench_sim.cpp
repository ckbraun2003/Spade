// Manual-only benchmark harness for the v2 engine's Simulation::step()
// throughput -- Task 21 close-out.
//
// spade_bench (see tests/CMakeLists.txt) is a standalone executable,
// NOT registered with ctest / gtest_discover_tests -- it is not part of the
// "spade" CTest label set the gate runs (scripts/test.ps1). google-benchmark drives it because the goal here is throughput
// measurement, not pass/fail assertions -- exactly bench_core.cpp's policy,
// restated for this file: correctness of everything exercised below (the
// integrator, the broad phase, the rotor/IMU chain) is covered separately by
// tests/test_*.cpp; this file is for "how fast."
//
// WHAT THIS FILE MEASURES, AND IN WHAT UNIT. Every benchmark below calls
// Simulation::step(n) with the Simulation configured at substeps == 1 (see
// build_plain_bodies_sim()/build_quad_worlds_sim()), so "n steps" and "n
// substeps" are the SAME n here -- step()'s own doc comment (sim/
// simulation.hpp) is explicit that a step runs the schedule `substeps` times,
// and substeps is 1 throughout this file. Every reported items_per_second is
// therefore STEPS/SEC, which at substeps==1 equals SUBSTEPS/SEC -- the unit
// engine design spec §12's rate-envelope targets are stated in. Nothing here
// measures wall-clock real-time-factor; that would require pairing this
// against the sim's own dt_ns, which is a config value, not a throughput
// number.
//
// THE TWO SWEEPS, AND WHY THEY ARE SHAPED AS THEY ARE.
//   * plain bodies (1 world x {10, 100, 1000} bodies, no vehicles, no SDF
//     geometry): general step-loop throughput as a function of body count --
//     integrator + the CollisionDynamic broad phase's grid bucketing, with
//     nothing to collide against (a void world -- CollisionStatic's SDF query
//     is the empty-program early-out) and no force elements. This is NOT the
//     spec's "quadrotor-class scene" target; it exists to show how the
//     per-substep cost scales with N before vehicles are anywhere in the
//     picture. READ THE THREE POINTS AS THREE SEPARATE MEASUREMENTS, NOT A
//     CLEAN N-SCALING CURVE: google-benchmark auto-selects each point's
//     iteration count to hit its own wall-clock budget, and a cheaper
//     per-substep cost buys MORE total substeps within that budget -- N=10
//     ran roughly 56 s of simulated time (free fall reaches ~549 m/s by
//     then) while N=1000 ran roughly 0.25 s (bodies barely moved off their
//     spawn grid). The three points are therefore each measuring a
//     DIFFERENT dynamical regime, not just a different N, so the apparent
//     superlinear falloff from N=10 to N=1000 partly reflects that
//     confound, not a pure per-substep cost law -- a future reader
//     comparing these numbers run-to-run as a regression signal should
//     account for it rather than read a moved number as a finding on its
//     own.
//   * quad worlds ({1, 4, 16, 64} worlds, ONE Quadrotor + ONE ideal IMU per
//     world, all in hover trim): this IS the spec's target shape.
//     spade::replicate() (sim/world_set.hpp) builds the N-world set from one
//     prototype instance, which is also what puts every world under
//     WorldSetLayout::uniform_dynamic_params -- the BATCHED CollisionDynamic
//     path (D8: "one dispatch steps N worlds"), the form §12's 64-world
//     target is actually about. Each world's counters{} entry additionally
//     reports "world_substeps_per_sec" (== items_per_second * world_count) --
//     the aggregate throughput figure §12's 64-world target is phrased in.
//
// THE AIRFRAME. bench_sim.cpp cannot include tools/viewer/scenes.cpp's
// demo_quadrotor_params(): that file compiles into spade_viewer, which is
// gated behind SPADE_BUILD_V1 (engine/CMakeLists.txt) and is therefore absent
// entirely from the Linux CI configuration this task's PR turns on. The
// values in bench_quadrotor_params() below are copied from
// tools/viewer/scenes.cpp's demo_quadrotor_params() (same mass, inertia, arm
// length, rotor constants, drag) rather than shared, so this file has no
// build-graph dependency on the viewer at all -- see CMakeLists.txt's
// SPADE_BUILD_V1 comment for why that separation exists.
//
// ---------------------------------------------------------------------------
// THE THIRD AND FOURTH SWEEPS (S6 Task 10): BM_StepPlainBodiesGpu and
// BM_StepQuadWorldsGpu -- the SAME two scenes and the SAME builders as above,
// the only difference being the BackendDesc each build_*_sim() call is handed:
// {kind = vulkan}, workgroup_size defaulted to 64 (compute/backend.hpp's own
// default, and the size every kernel was authored/parity-banded at through S6
// Tasks 6-8 -- see baselines.json's _meta for why this sweep does not also
// vary it). Skips gracefully -- state.SkipWithError() -- on any setup failure,
// by catching build_plain_bodies_sim()/build_quad_worlds_sim()'s unwrap()
// throw rather than pre-checking compute::vulkan_available() directly: a box
// with no working vulkan path makes Simulation::create() fail with
// Code::unavailable regardless of WHY (no device, or SPADE_VULKAN=OFF --
// compute/vulkan/backend_stub.cpp's own header note is explicit that those are
// "the SAME fact" from a caller's perspective), and catching that one throw
// handles both without this file including compute/vulkan/context.hpp
// directly. That distinction is not stylistic: unlike spade_tests (every
// GPU-touching test file is added to that target's sources only inside an
// `if(SPADE_VULKAN)` guard, so a Vulkan header include inside one of them is
// simply never compiled in an OFF build), spade_bench compiles this file
// UNCONDITIONALLY, so an unconditional Vulkan-header include here would break
// the SPADE_VULKAN=OFF configuration -- volk/Vulkan::Headers are not on this
// file's include path at all in that build.
//
// PER-PASS COUNTERS. Each GPU benchmark also reads
// Simulation::vulkan_pass_durations_ns() once, after the timed loop. Since
// module-API stage 2 that is one named duration per GPU pass of the compiled
// schedule, and it is reported two ways, each in nanoseconds:
//
//   * ONE COUNTER PER PASS, gpu_pass.<module>.<pass>_ns, generated from the
//     list -- so a new module's pass shows up here without anyone editing this
//     file, and the two behavior passes (which record nothing) show their
//     measured ~0 rather than being left out.
//   * THE SIX HISTORICAL COUNTERS, kept by name because bench/baselines.json
//     records 42 values under them: gpu_medium_update_ns, gpu_force_elements_ns
//     (rotor.forces + drag.forces), gpu_collision_static_ns,
//     gpu_collision_dynamic_ns, gpu_integrate_ns and gpu_sensor_synthesis_ns
//     (imu.synthesize + gnss.synthesize). The two sums each include one more
//     timestamp mark's cost than before stage 2, when each pair shared a
//     bracket (TD-8).
//
// A PASS ONE OF THE SIX NEEDS THAT IS NOT IN THE LIST IS AN ERROR, NOT A
// ZERO (TD-5). Every built-in pass is always in the list -- it comes from the
// schedule, not from row counts -- so absence means a module or pass was
// renamed, and the counter would otherwise silently read 0 or lose a term.
//
// WHAT THE COUNTERS MEASURE, PRECISELY. compute/vulkan/timestamps.hpp's
// PassTimestamps resets and rewrites its query pool on EVERY GPU submit
// (StepRecorder's "record once, submit n times" chain resubmits the same
// timestamp-writing commands along with the physics dispatches), so by the
// time the timed `for (auto _ : state)` loop above has finished, the query
// pool holds only the LAST of the many GPU submits that loop performed -- one
// representative step's per-pass breakdown, not a sum or average over the
// whole benchmark run. That is the right number to report for these two
// scenes: every step does IDENTICAL work (hover trim, free fall with no
// contacts), so one step's breakdown is the steady-state answer, and reading
// it once outside the timed region costs nothing the timed measurement itself
// would have to account for.
//
// A GPU family's items_per_second/world_substeps_per_sec are computed on the
// SAME cpu_time/real_time basis as the CPU families above (SetItemsProcessed()
// and the world_substeps_per_sec counter do not know or care which backend
// produced the step() calls they are timing) -- so the round trip they measure
// includes the fence waits StepRecorder::submit() blocks on per substep, not
// device-execution time alone. The per-pass counters are the device-side
// complement: PURE GPU EXECUTION time, no host wait folded in, which is what
// makes "device work vs. round-trip overhead" a comparison these two numbers
// together can actually answer.
// ---------------------------------------------------------------------------

// Baseline comparison policy (through S6): tests/bench/baselines.json
// holds recorded runs as REFERENCE points only -- see that file's own
// "_meta" entry and bench_core.cpp's header comment for the full policy.
// Nothing in CI reads or diffs against it.
//
// Run (after scripts/build.ps1, or the equivalent direct-configure
// build on Linux):
//   build-ninja/release/bin/spade_bench.exe --benchmark_format=json
//   build-ci/bin/spade_bench --benchmark_format=json
//
// Never invoke this (or any other Spade binary) from a backgrounded shell on
// this box -- run it in the foreground, same as bench_core.cpp and every
// other manual tool in this tree.

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace {

using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::ModelTypeId;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::VehicleSpawn;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::PassDurationsNs;

// A benchmark's setup runs OUTSIDE the timed region, but a setup failure
// still has to stop the run loudly rather than silently benchmark garbage --
// same "cannot fail here, but check anyway" posture as tools/viewer/
// scenes.cpp's unwrap_world()/unwrap_model(), restated locally so this file
// has no dependency on that one. Deliberately NOT [[nodiscard]]: several call
// sites below use this purely for its throw-on-failure side effect (spawn()'s
// returned ref, flush_structural()'s Result<void>) and have nothing to do
// with the value. `if constexpr` covers Result<void> too -- plain
// `return std::move(*result)` does not compile for T = void.
template <class T>
T unwrap(spade::Result<T> result, const char* what) {
    if (!result) {
        throw std::runtime_error(std::string("bench_sim setup: ") + what + ": " + result.error().context);
    }
    if constexpr (!std::is_void_v<T>) {
        return std::move(*result);
    }
}

// One step call advances `substeps` (== 1 throughout this file, see the file
// header) times `kStepsPerIter`. Batched per google-benchmark iteration to
// amortize the harness's own loop overhead relative to the step cost being
// measured -- the same reasoning bench_core.cpp's single-call-per-iteration
// shape does not need (integrate_orientation() is cheap enough that harness
// overhead is negligible there; a full Simulation::step() is not).
constexpr uint64_t kStepsPerIter = 50;

// Every scene here runs at the same nominal 1 kHz step rate the demo scenes
// and test suites use (dt_ns = 1 ms, substeps = 1 -> h = 1 ms) -- see
// tests/test_quadrotor.cpp's void_sim()/test_m1b_bar.cpp's quadrotor_imu_
// smoke(). The dt/substeps split does not change the FLOP count of a
// substep; it only scales `h`, so this choice affects nothing about
// throughput and everything about matching the rest of the tree's
// convention.
constexpr uint64_t kDtNs = 1'000'000;
constexpr uint32_t kSubsteps = 1;

// S6 Task 10: shared by both GPU families below (BM_StepPlainBodiesGpu,
// BM_StepQuadWorldsGpu) -- reads Simulation::vulkan_pass_durations_ns() and,
// if the query succeeded and this device could time compute work, sets one
// counter per pass and the six historical sums. See the file header's
// "PER-PASS COUNTERS" section for the unit (nanoseconds), why a missing pass
// is an error, and what "reads the LAST GPU submit" means for these two
// steady-state scenes.
//
// A FAILED OR UNSUPPORTED READING SETS NO COUNTERS AT ALL, rather than
// forcing zeros into the JSON output: `!d` means a real Vulkan error reading
// the query pool back (state.SkipWithError(), matching this file's existing
// setup-failure posture for anything that can fail without being part of the
// timed region); `d->supported == false` means this device/queue could not
// time compute work (compute/vulkan/timestamps.hpp's skip-gracefully
// posture) -- not expected on this program's correctness device (Intel Iris
// Plus), but a caller-visible fact rather than a silently-reported 0 either
// way.
void set_pass_duration_counters(benchmark::State& state, Simulation& sim) {
    const spade::Result<PassDurationsNs> d = sim.vulkan_pass_durations_ns();
    if (!d) {
        state.SkipWithError(d.error().context.c_str());
        return;
    }
    if (!d->supported) {
        return;
    }

    // Every name the six sums need, checked BEFORE any counter is set, so a
    // rename skips the benchmark with its name rather than reporting a partial
    // or zero breakdown.
    constexpr const char* kNeeded[] = {"dryden.advance",          "rotor.forces",        "drag.forces",
                                       "static_contact.resolve",  "dynamic_contact.resolve",
                                       "integrate.integrate",     "imu.synthesize",      "gnss.synthesize"};
    for (const char* name : kNeeded) {
        if (!d->find(name).has_value()) {
            state.SkipWithError(("pass '" + std::string(name) + "' is not in the recorded chain").c_str());
            return;
        }
    }
    const auto ns = [&](const char* name) { return d->find(name).value(); };  // present: checked above

    for (const spade::compute::PassDuration& pass : d->passes) {
        state.counters["gpu_pass." + pass.pass + "_ns"] = pass.ns;
    }
    state.counters["gpu_medium_update_ns"] = ns("dryden.advance");
    state.counters["gpu_force_elements_ns"] = ns("rotor.forces") + ns("drag.forces");
    state.counters["gpu_collision_static_ns"] = ns("static_contact.resolve");
    state.counters["gpu_collision_dynamic_ns"] = ns("dynamic_contact.resolve");
    state.counters["gpu_integrate_ns"] = ns("integrate.integrate");
    state.counters["gpu_sensor_synthesis_ns"] = ns("imu.synthesize") + ns("gnss.synthesize");

    // L307 (2): THE INSTRUMENT'S OWN HEALTH, BESIDE THE NUMBERS IT PRODUCED.
    // Non-zero means at least one pass sample exceeded kImplausibleSampleNs
    // and every duration above is suspect. Emitted unconditionally -- a
    // health counter that only appears when it is bad is one a reader cannot
    // distinguish from a build that never reported it, and "the field is
    // missing" reads as "fine" to every consumer.
    state.counters["gpu_implausible_samples"] = static_cast<double>(d->implausible_samples);

}

// ---------------------------------------------------------------------------
// Sweep 1: 1 world x {10, 100, 1000} plain bodies, no vehicles, no SDF
// geometry (a void world -- see the file header).
// ---------------------------------------------------------------------------

// `backend` (S6 Task 10): DEFAULTED to {} (BackendKind::cpu), so every
// pre-Task-10 call site -- BM_StepPlainBodies below -- is unaffected.
// BM_StepPlainBodiesGpu passes BackendDesc{BackendKind::vulkan} instead,
// same scene, same builder, different Simulation::create() backend argument
// -- exactly the split simulation.hpp's own create() doc comment describes.
[[nodiscard]] Simulation build_plain_bodies_sim(uint32_t body_count, const BackendDesc& backend = {}) {
    const spade::WorldDesc world = unwrap(WorldBuilder()
                                               .name("bench_plain_bodies")
                                               .environment(Environment{})
                                               .capacities(Capacities{body_count, 1, 1, 1})
                                               .build(),
                                           "build plain-bodies world");

    WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 0xB0D1E000ULL + body_count;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.3f;
    instance.contacts.friction_mu = 0.5f;
    instance.contacts.proxy_radius = 0.2f;
    instance.grid.cell_size = 0.5f;  // >= 2 * proxy_radius

    Simulation sim = unwrap(Simulation::create(WorldSetDesc{{instance}}, kDtNs, kSubsteps, backend),
                             "Simulation::create (plain bodies)");

    // Spread across an XZ grid, well clear of the ground (there is none --
    // void world) and spaced 2 m apart so the broad phase buckets them into
    // distinct cells rather than one degenerate pile -- this sweep is about
    // general step-loop scaling with N, not about stressing Gauss-Seidel
    // contact resolution (that is the `shower` scenario's job in the
    // determinism corpus).
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(body_count))));
    for (uint32_t i = 0; i < body_count; ++i) {
        const uint32_t row = i / side;
        const uint32_t col = i % side;
        BodySpawn body;
        body.pos = glm::vec3(static_cast<float>(col) * 2.0f, 50.0f, static_cast<float>(row) * 2.0f);
        body.mass = 1.0f;
        body.inv_inertia_diag = glm::vec3(1.0f);
        unwrap(sim.spawn(0, body), "spawn plain body");
    }
    unwrap(sim.flush_structural(), "flush_structural (plain bodies)");
    return sim;
}

void BM_StepPlainBodies(benchmark::State& state) {
    const uint32_t body_count = static_cast<uint32_t>(state.range(0));
    Simulation sim = build_plain_bodies_sim(body_count);

    for (auto _ : state) {
        const spade::Result<void> r = sim.step(kStepsPerIter);
        if (!r) {
            state.SkipWithError(r.error().context.c_str());
            break;
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(kStepsPerIter));
}
BENCHMARK(BM_StepPlainBodies)->Arg(10)->Arg(100)->Arg(1000);

// S6 Task 10: Sweep 1's GPU twin -- see the file header's "THE THIRD AND
// FOURTH SWEEPS" section (in particular for why this catches unwrap()'s
// throw instead of pre-checking compute::vulkan_available()). Same scene,
// same builder, BackendDesc{vulkan} (workgroup_size defaulted to 64).
void BM_StepPlainBodiesGpu(benchmark::State& state) {
    try {
        const uint32_t body_count = static_cast<uint32_t>(state.range(0));
        Simulation sim = build_plain_bodies_sim(body_count, BackendDesc{BackendKind::vulkan});

        for (auto _ : state) {
            const spade::Result<void> r = sim.step(kStepsPerIter);
            if (!r) {
                state.SkipWithError(r.error().context.c_str());
                break;
            }
        }
        state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(kStepsPerIter));

        set_pass_duration_counters(state, sim);
    } catch (const std::runtime_error& e) {
        state.SkipWithError(e.what());
    }
}
// ⚠ THIS NAME DOES NOT SAY "BATCHED" AND IT SHOULD HAVE. Every case above
// calls sim.step(kStepsPerIter) -- ONE step(50) per iteration -- so it
// measures the BATCHED path: 50 fences but only ONE readback, amortised.
// Production never does this. All four production call sites pass n = 1
// (sdk/core/rollout.cpp:882 and :948, editor/core/sim/tick_loop.cpp:259 and
// :286), paying a full readback EVERY tick.
//
// Annotated rather than RENAMED, deliberately: a rename would silently
// zero-match every stored --benchmark_filter and every habit pointing at
// this name, and a filter that matches nothing is a confident green on an
// empty set. Same defect the estate hit tonight when a ctest label was
// renamed to a string that CONTAINED the old one.
BENCHMARK(BM_StepPlainBodiesGpu)->Arg(10)->Arg(100)->Arg(1000);

// ---------------------------------------------------------------------------
// Sweep 1c: THE SAME SCENE THROUGH THE CALL SHAPE PRODUCTION ACTUALLY USES.
//
// WHY THIS IS A SECOND ARM AND NOT A SECOND ARGUMENT TO THE FIRST. The two
// differ only in how the same 50 steps are requested:
//
//     BM_StepPlainBodiesGpu            step(50) once    50 fences, 1 readback
//     BM_StepPlainBodiesGpuUnbatched   step(1) 50 times 50 fences, 50 readbacks
//
// simulation.cpp:620-641 calls submit(n) ONCE and readback() ONCE per
// step(n), while submit(n) itself waits on a fence PER STEP. So batching
// divides the readback by 50 and leaves the fences alone -- and the
// difference between these two arms is precisely the per-step cost that
// batching hides and that every production tick pays.
//
// ⭐ THAT MAKES THIS PAIR ITS OWN CONTROL. Sizing the per-step stall by
// subtracting summed pass counters from a wall-clock figure does not work:
// the remainder silently absorbs readback, host bookkeeping and anything
// else unaccounted, and a subtraction cannot be attributed to one of its
// terms. Two arms differing in exactly one variable can.
//
// 🔴 THIS PAIR WAS BUILT AS L307 (2)'s FALSIFIER AND IT IS NOT ONE. RECORDED
// HERE RATHER THAN QUIETLY RESCOPED, BECAUSE THE MISTAKE IS THE USEFUL PART.
//
// The intent was: the difference between the arms is what the params ring
// removes. It is not. simulation.cpp:620-641 calls backend->step(n) ONCE and
// readback() ONCE per step(n), while submit(n) fences PER STEP -- so:
//
//     batched     50 fences +  1 readback
//     unbatched   50 fences + 50 readbacks
//
// BOTH ARMS PAY FIFTY FENCES. The pair isolates the READBACK, which is what
// BATCHING removes. The ring removes the FENCE, which is identical in both
// arms and therefore cancels exactly. A controlled pair differing in the
// wrong variable answers a question nobody asked, confidently.
//
// Measured anyway, and it is a real fact about the system:
//     Arg(10)   1.34 -> 1.75 ms/step   1.30x
//     Arg(100)  9.37 -> 9.17 ms/step   0.98x   <- sign flips; noise at n=1
//     Arg(1000) 102.3 -> 103.0 ms/step 1.01x
// So batching buys ~0.4 ms/step at editor scale and nothing at all beyond it.
// KEEP THIS PAIR FOR THAT -- it is the only measurement of the gap between
// the batched path the benchmarks use and the n=1 path all four production
// call sites take. It is simply not the ring's falsifier.
//
// The ring's own measurement does NOT exist yet, and backend.hpp records
// why: the host-side timer that seemed obvious is forbidden in engine
// source by the fixed-step determinism guard, AND it measures GPU execution
// rather than stall. Sizing the ring needs the device-timeline gap between
// steps, which needs the per-slot query pool the ring itself introduces.
//
// ---------------------------------------------------------------------------
// THE DECISION THRESHOLDS, PRE-REGISTERED BEFORE THE FIRST RUN -- AND NOT
// APPLIED, BECAUSE THE INSTRUMENT TURNED OUT NOT TO MEASURE THEIR SUBJECT.
// ---------------------------------------------------------------------------
// Kept verbatim rather than deleted. They were written before any number
// existed and they did their job: when the run came back at 1.30x -- inside
// the "report and ask" band -- the pressure was to read that as a verdict.
// What stopped it was not the band but the discovery above that both arms
// fence 50 times, so the ratio is about readback and these thresholds are
// about a quantity this pair cannot see.
//
// Written into the source rather than a report, because a threshold that
// arrives with the result is not a threshold -- it is a reading of the
// result. `git log` shows this paragraph predates the numbers it judges.
//
// ⚠ A PRE-REGISTERED THRESHOLD DOES NOT MAKE AN INSTRUMENT VALID. It only
// stops you moving the line after seeing the number. Both checks are needed,
// and they fail independently.
//
//   unbatched >= 1.5x batched, per step, at Arg(10)
//       BUILD THE RING. An editor tick is paying more for the fence and the
//       readback than for the physics.
//
//   the two arms within 1.2x
//       DO NOT BUILD IT, and that is the deliverable rather than a failure.
//       R command buffers, R fences and a per-slot query pool are real risk
//       against a stall that is not there.
//
//   between 1.2x and 1.5x
//       REPORT THE NUMBER AND ASK. Deciding in that band, having already
//       written the arm, is deciding under the influence of the work
//       already done.
//
// Arg(100) and Arg(1000) run too because they are free, but THE DECISION
// RIDES ON Arg(10): the editor viewport is a handful of bodies at n = 1 and
// it is the front-facing case. The others are context, not the verdict.
// ---------------------------------------------------------------------------
void BM_StepPlainBodiesGpuUnbatched(benchmark::State& state) {
    try {
        const uint32_t body_count = static_cast<uint32_t>(state.range(0));
        Simulation sim = build_plain_bodies_sim(body_count, BackendDesc{BackendKind::vulkan});

        for (auto _ : state) {
            // n = 1, fifty times: what rollout.cpp and tick_loop.cpp do.
            bool failed = false;
            for (uint64_t i = 0; i < kStepsPerIter; ++i) {
                const spade::Result<void> r = sim.step(1);
                if (!r) {
                    state.SkipWithError(r.error().context.c_str());
                    failed = true;
                    break;
                }
            }
            if (failed) break;
        }
        state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) *
                                 static_cast<int64_t>(kStepsPerIter));

        set_pass_duration_counters(state, sim);
    } catch (const std::runtime_error& e) {
        state.SkipWithError(e.what());
    }
}
// Same arguments as the batched arm, so the pair is comparable row for row.
//
// ⚠ THE PAIR IS NOT FLAG-IDENTICAL AND THE READER MUST KNOW WHY. This arm
// carries ->UseRealTime() and BM_StepPlainBodiesGpu above does not. That is
// one more difference than a controlled pair is allowed, and it exists
// because a host blocked in vkWaitForFences accrues almost no CPU time, so
// google-benchmark's CPU-based iteration heuristic will happily run this arm
// for a very long time without it.
//
// UseRealTime() changes only the ITERATION HEURISTIC -- the reported Time
// column is wall clock in both cases -- so the per-step figures are
// comparable. But rather than rely on that, RUN BOTH ARMS WITH
// --benchmark_min_time=1x, which pins each to exactly one iteration and
// removes the heuristic from the comparison entirely. Then the only
// remaining difference between the two arms is the call shape, which is what
// the pair exists to isolate.
//
// Adding UseRealTime() to the batched arm instead would have equalised the
// flags, and it is NOT done here: that arm's reported numbers are quoted
// elsewhere, and changing how an existing benchmark measures is a moved
// pinned number wearing a cleanup's clothes.
BENCHMARK(BM_StepPlainBodiesGpuUnbatched)->Arg(10)->Arg(100)->Arg(1000)->UseRealTime();

// ---------------------------------------------------------------------------
// Sweep 1b: THE CONTACT FALSIFIER -- the same bodies, packed so they TOUCH.
//
// ⭐⭐⭐ WHY THIS EXISTS, AND IT IS THE WHOLE REASON IT LANDS BEFORE ANY KERNEL
// CHANGE. Sweep 1 above spaces bodies 2 m apart with a 0.4 m contact distance
// (proxy_radius 0.2), so NOTHING EVER TOUCHES: `resolve_pair` returns at its
// first test every time and 100% of the measured cost is a search that finds
// nothing. A fix to the collision kernel measured against that fixture reports
// a total win WHATEVER IT DOES, INCLUDING NOTHING.
//
// ***A BENCHMARK THAT CANNOT DISTINGUISH A FIX FROM A NO-OP IS NOT EVIDENCE
// FOR EITHER, AND THE DIRECTION OF ITS ERROR IS FLATTERING.***
//
// ⚠ AND NO EXISTING FIXTURE ON THIS ESTATE CAN STAND IN: every world in
// content/worlds/ declares capacities.bodies = 1 except swarm-grid, which has
// 4. `collision_dynamic` is body-vs-body, so one body is no pair at all --
// the shipped corpus provably cannot exercise it.
//
// ⭐⭐ THE PAIRED CONTROL IS THE POINT, NOT A GARNISH. This benchmark takes
// SPACING as its second argument, so the identical scene runs dense and sparse
// at the same body count. If the two do not differ, the fixture is not
// measuring contact work and every number after it is worthless -- the same
// shape as "10 boxes cost the same as 1 box" needing "3 shapes cost more" to
// mean anything.
//
// WHY THE PILE STAYS PUT, which a contact benchmark has to get right or it
// measures a different scene on every iteration: gravity is ZEROED and
// restitution is ZERO. Bodies start at rest, overlap by construction, and
// `resolve_pair` writes only velocity -- so with no gravity to drive them and
// no bounce to separate them, the contact SET is constant for the whole run.
// A pile that explodes apart would make the first iterations dense, the last
// ones sparse, and the mean meaningless.
[[nodiscard]] Simulation build_dense_bodies_sim(uint32_t body_count, float spacing,
                                                const BackendDesc& backend = {}) {
    Environment env;
    env.gravity = glm::vec3(0.0f);  // see above -- keeps the contact set constant

    const spade::WorldDesc world = unwrap(WorldBuilder()
                                               .name("bench_dense_bodies")
                                               .environment(env)
                                               .capacities(Capacities{body_count, 1, 1, 1})
                                               .build(),
                                           "build dense-bodies world");

    WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 0xD0FFE000ULL + body_count;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;  // no bounce -> the pile does not separate
    instance.contacts.friction_mu = 0.5f;
    instance.contacts.proxy_radius = 0.2f;   // contact distance 0.4 m
    instance.grid.cell_size = 0.5f;          // >= 2 * proxy_radius

    Simulation sim = unwrap(Simulation::create(WorldSetDesc{{instance}}, kDtNs, kSubsteps, backend),
                             "Simulation::create (dense bodies)");

    // A CUBE, not a plane. A 2D grid at close spacing gives each body 4
    // neighbours; a 3D packing gives it 6, and the point of this fixture is to
    // put real work into the narrow phase rather than to look dense.
    const uint32_t side =
        static_cast<uint32_t>(std::ceil(std::cbrt(static_cast<double>(body_count))));
    for (uint32_t i = 0; i < body_count; ++i) {
        const uint32_t x = i % side;
        const uint32_t y = (i / side) % side;
        const uint32_t z = i / (side * side);
        BodySpawn body;
        body.pos = glm::vec3(static_cast<float>(x), static_cast<float>(y),
                             static_cast<float>(z)) *
                   spacing;
        body.mass = 1.0f;
        body.inv_inertia_diag = glm::vec3(1.0f);
        unwrap(sim.spawn(0, body), "spawn dense body");
    }
    unwrap(sim.flush_structural(), "flush_structural (dense bodies)");
    return sim;
}

// range(0) = body count, range(1) = spacing in MILLIMETRES (benchmark args are
// integers). 300 mm is inside the 400 mm contact distance; 2000 mm is Sweep
// 1's spacing and is the CONTROL -- it must cost measurably less, or this
// fixture is not doing what it claims.
void BM_StepDenseBodies(benchmark::State& state) {
    const uint32_t body_count = static_cast<uint32_t>(state.range(0));
    const float spacing = static_cast<float>(state.range(1)) * 0.001f;
    Simulation sim = build_dense_bodies_sim(body_count, spacing);

    for (auto _ : state) {
        const spade::Result<void> r = sim.step(kStepsPerIter);
        if (!r) {
            state.SkipWithError(r.error().context.c_str());
            break;
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) *
                            static_cast<int64_t>(kStepsPerIter));
}
BENCHMARK(BM_StepDenseBodies)
    ->Args({100, 300})
    ->Args({100, 2000})
    ->Args({1000, 300})
    ->Args({1000, 2000})
    ->UseRealTime();

void BM_StepDenseBodiesGpu(benchmark::State& state) {
    try {
        const uint32_t body_count = static_cast<uint32_t>(state.range(0));
        const float spacing = static_cast<float>(state.range(1)) * 0.001f;
        Simulation sim = build_dense_bodies_sim(body_count, spacing, BackendDesc{BackendKind::vulkan});

        for (auto _ : state) {
            const spade::Result<void> r = sim.step(kStepsPerIter);
            if (!r) {
                state.SkipWithError(r.error().context.c_str());
                break;
            }
        }
        state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) *
                                static_cast<int64_t>(kStepsPerIter));
        set_pass_duration_counters(state, sim);
    } catch (const std::runtime_error& e) {
        state.SkipWithError(e.what());
    }
}
// ⚠ ->UseRealTime() IS LOAD-BEARING ON THE GPU ROWS AND ITS ABSENCE ALREADY
// PRODUCED ONE WRONG CONCLUSION IN THIS FILE'S HISTORY. Without it, google
// benchmark reports CPU time, and a host blocked in vkWaitForFences accrues
// almost none -- so the GPU rows read as near-parity with the CPU and one row
// reported inf/s. Only wall clock compares these two backends.
BENCHMARK(BM_StepDenseBodiesGpu)
    ->Args({100, 300})
    ->Args({100, 2000})
    ->Args({1000, 300})
    ->Args({1000, 2000})
    ->UseRealTime();

// ---------------------------------------------------------------------------
// Sweep 2: {1, 4, 16, 64} worlds x the quad scene -- one Quadrotor + one
// ideal IMU per world, spawned IN TRIM (hover_command()), all worlds sharing
// one ContactParams/GridParams so replicate() puts the set on the batched
// CollisionDynamic path (D8). This is the shape engine design spec §12's
// rate-envelope targets describe.
// ---------------------------------------------------------------------------

// Copied from tools/viewer/scenes.cpp's demo_quadrotor_params() -- see the
// file header comment for why this is a copy, not a shared include. Same
// airframe: 1 kg, three distinct principal moments, 18 cm arms, 13 cm props.
[[nodiscard]] spade::vehicles::QuadrotorParams bench_quadrotor_params() {
    spade::vehicles::QuadrotorParams p;
    p.name = "bench_quad";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    p.rotor_height = 0.02f;
    p.proxy_radius = 0.2f;
    for (spade::vehicles::RotorParams& rotor : p.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.13f;
        rotor.thrust_coeff = 1.2e-5f;
        rotor.torque_coeff = 1.9e-7f;
    }
    p.drag.mode = spade::physics::drag_mode::quadratic;
    p.drag.area = 0.05f;
    p.drag.coeffs = glm::vec3(1.6f, 0.0f, 0.0f);
    p.imu.rate_divider = 1;  // ideal sensor, every substep
    return p;
}

// `backend` (S6 Task 10): same defaulted-trailing-parameter shape as
// build_plain_bodies_sim() above, for the identical reason.
[[nodiscard]] Simulation build_quad_worlds_sim(uint32_t world_count, const BackendDesc& backend = {}) {
    const spade::vehicles::QuadrotorParams params = bench_quadrotor_params();

    const spade::WorldDesc ground = unwrap(WorldBuilder()
                                                .name("bench_quad_ground")
                                                .environment(Environment{})
                                                .capacities(Capacities{1, 5, 1, 1})  // 4 rotors + 1 drag element
                                                .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                .build(),
                                            "build quad-scene ground world");

    WorldInstanceDesc prototype;
    prototype.world = ground;
    prototype.turbulence = spade::dryden_params(TurbulenceLevel::none);
    prototype.contacts.restitution_e = 0.2f;
    prototype.contacts.friction_mu = 0.5f;
    prototype.contacts.proxy_radius = 0.2f;
    prototype.grid.cell_size = 0.5f;  // >= 2 * proxy_radius

    const WorldSetDesc set = spade::replicate(prototype, world_count, 0x51'4C'A9'E5ULL);
    Simulation sim =
        unwrap(Simulation::create(set, kDtNs, kSubsteps, backend), "Simulation::create (quad worlds)");

    const spade::vehicles::ModelType model = unwrap(spade::vehicles::make_quadrotor(params), "make_quadrotor");
    const ModelTypeId model_id = unwrap(sim.register_model(model), "register_model");

    const float hover = spade::vehicles::hover_command(params);
    for (uint32_t w = 0; w < world_count; ++w) {
        VehicleSpawn where;
        where.pos = glm::vec3(0.0f, 3.0f, 0.0f);
        where.rotor_omega = hover;  // IN TRIM -- see scene_hover()'s comment
        unwrap(sim.spawn(w, model_id, where), "spawn quad vehicle");
    }
    unwrap(sim.flush_structural(), "flush_structural (quad worlds)");
    return sim;
}

void BM_StepQuadWorlds(benchmark::State& state) {
    const uint32_t world_count = static_cast<uint32_t>(state.range(0));
    Simulation sim = build_quad_worlds_sim(world_count);

    for (auto _ : state) {
        const spade::Result<void> r = sim.step(kStepsPerIter);
        if (!r) {
            state.SkipWithError(r.error().context.c_str());
            break;
        }
    }
    state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(kStepsPerIter));

    // Aggregate throughput across the whole batched set, in the unit §12's
    // 64-world target is phrased in (world-substeps/sec). benchmark::Counter
    // with kIsRate divides its VALUE by the run's measured TIME -- and
    // neither this benchmark nor BM_StepPlainBodies calls ->UseRealTime(),
    // so that measured time is CPU time, not wall-clock real time (same as
    // items_per_second above, which google-benchmark also computes from
    // cpu_time by default -- SetItemsProcessed()'s own contract does not
    // change that). Matching SetItemsProcessed()'s TOTAL-not-per-iteration
    // contract, the value handed in here is the TOTAL accumulated over
    // every iteration -- world_count times the same
    // iterations() * kStepsPerIter total items_per_second is computed from
    // -- which is what makes this counter and items_per_second agree on the
    // underlying substeps/sec and differ only by the world_count factor.
    //
    // THE BASIS MATTERS FOR THE SPEC COMPARISON. §12's single-world target is
    // phrased "CPU substeps", so a CPU-time-based rate is the RIGHT
    // comparison for it. §12's batched target is phrased "faster than real
    // time", which is inherently a wall-clock claim -- a CPU-time-based rate
    // is the wrong basis to cite for THAT one on its own terms. See
    // baselines.json's spec_comparison for the honest fix: both targets
    // recomputed on real_time (this run's raw context.benchmarks[].real_time
    // field, not anything this counter reports) still clear their floors --
    // ~28.6x and ~2.80x respectively -- so the verdict does not change, but
    // the number cited for it should be the one actually measured on the
    // basis the target's own words describe.
    state.counters["world_substeps_per_sec"] = benchmark::Counter(
        static_cast<double>(state.iterations()) * static_cast<double>(kStepsPerIter) *
            static_cast<double>(world_count),
        benchmark::Counter::kIsRate);
}
BENCHMARK(BM_StepQuadWorlds)->Arg(1)->Arg(4)->Arg(16)->Arg(64);

// S6 Task 10: Sweep 2's GPU twin, and THE FIRST TIME §12's "batched headless
// >= 64 worlds x 1 kHz faster than real time on the dev GPU" target is
// measurable on the literal hardware it names -- baselines.json's existing
// _meta calls the CPU family above only "a strong signal ... not the literal
// target measurement" for exactly this reason. See baselines.json's own
// _meta for this run's verdict (RECORDED, NOT GATED -- a dev-GPU miss on this
// iGPU is a user-ratified posture, not a build failure) and for training spec
// TR12's second-consumer note on the same numbers.
//
// world_substeps_per_sec here is computed IDENTICALLY to the CPU family's
// (same formula, same TOTAL-not-per-iteration contract) -- on cpu_time by
// default, same as items_per_second, which is the round-trip basis (fence
// waits included) rather than device-execution time alone; see this file's
// header for why that is the right complement to the per-pass counters
// rather than a competing "the real number" claim.
void BM_StepQuadWorldsGpu(benchmark::State& state) {
    try {
        const uint32_t world_count = static_cast<uint32_t>(state.range(0));
        Simulation sim = build_quad_worlds_sim(world_count, BackendDesc{BackendKind::vulkan});

        for (auto _ : state) {
            const spade::Result<void> r = sim.step(kStepsPerIter);
            if (!r) {
                state.SkipWithError(r.error().context.c_str());
                break;
            }
        }
        state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(kStepsPerIter));

        state.counters["world_substeps_per_sec"] = benchmark::Counter(
            static_cast<double>(state.iterations()) * static_cast<double>(kStepsPerIter) *
                static_cast<double>(world_count),
            benchmark::Counter::kIsRate);

        set_pass_duration_counters(state, sim);
    } catch (const std::runtime_error& e) {
        state.SkipWithError(e.what());
    }
}
BENCHMARK(BM_StepQuadWorldsGpu)->Arg(1)->Arg(4)->Arg(16)->Arg(64);

}  // namespace

// NO BENCHMARK_MAIN() HERE. spade_bench is ONE executable built from both
// this file and bench_core.cpp (tests/CMakeLists.txt); BENCHMARK_MAIN()
// expands to a `main()` definition, and google-benchmark's registration
// macros (BENCHMARK(...) above) register into a process-wide list via static
// initializers regardless of which translation unit runs main -- so a second
// BENCHMARK_MAIN() here would only produce an ODR violation (duplicate
// `main`, a link error caught at this task's own build verification), not
// additional coverage. bench_core.cpp supplies the one `main()` the binary
// needs; every BENCHMARK() registered from either file runs under it.
