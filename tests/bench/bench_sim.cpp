// Manual-only benchmark harness for the v2 engine's Simulation::step()
// throughput -- Task 21 close-out.
//
// spade_bench (see spade/tests/CMakeLists.txt) is a standalone executable,
// NOT registered with ctest / gtest_discover_tests -- it is not part of the
// "spade" or "T0" CTest label sets that CI runs, and is not expected to be
// until S6. google-benchmark drives it because the goal here is throughput
// measurement, not pass/fail assertions -- exactly bench_core.cpp's policy,
// restated for this file: correctness of everything exercised below (the
// integrator, the broad phase, the rotor/IMU chain) is covered separately by
// spade/tests/test_*.cpp; this file is for "how fast."
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
// build-graph dependency on the viewer at all -- see spade/CMakeLists.txt's
// SPADE_BUILD_V1 comment for why that separation exists.
//
// Baseline comparison policy (through S6): spade/tests/bench/baselines.json
// holds recorded runs as REFERENCE points only -- see that file's own
// "_meta" entry and bench_core.cpp's header comment for the full policy.
// Nothing in CI reads or diffs against it.
//
// Run (after spade/scripts/build.ps1, or the equivalent direct-configure
// build on Linux):
//   spade/build-ninja/release/bin/spade_bench.exe --benchmark_format=json
//   spade/build-ci/bin/spade_bench --benchmark_format=json
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

// ---------------------------------------------------------------------------
// Sweep 1: 1 world x {10, 100, 1000} plain bodies, no vehicles, no SDF
// geometry (a void world -- see the file header).
// ---------------------------------------------------------------------------

[[nodiscard]] Simulation build_plain_bodies_sim(uint32_t body_count) {
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

    Simulation sim = unwrap(Simulation::create(WorldSetDesc{{instance}}, kDtNs, kSubsteps),
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

[[nodiscard]] Simulation build_quad_worlds_sim(uint32_t world_count) {
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
    Simulation sim = unwrap(Simulation::create(set, kDtNs, kSubsteps), "Simulation::create (quad worlds)");

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
