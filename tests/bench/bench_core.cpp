// Manual-only benchmark harness for the v2 engine's core math primitives.
//
// spade_bench (see tests/CMakeLists.txt) is a standalone executable,
// NOT registered with ctest / gtest_discover_tests -- it is not part of the
// "spade" CTest label set the gate runs (scripts/test.ps1). google-benchmark drives it because the goal here is throughput
// measurement, not pass/fail assertions: correctness of
// spade::math::integrate_orientation is covered separately (closed-form
// comparisons in tests/test_math.cpp), this file is for "how fast."
//
// Baseline comparison policy (through S6): tests/bench/baselines.json
// holds recorded runs' numbers as REFERENCE points only, seeded by hand --
// see that file's own "_meta" entry for box/date/provenance (re-seeded more
// than once already; "first recorded run" stopped being true the moment a
// second one landed, so this comment does not claim it). Nothing in CI
// reads or diffs against it; there is no automated regression
// gate yet. A human re-runs this binary locally and eyeballs the new
// --benchmark_format=json numbers against baselines.json when investigating
// a suspected perf regression. S6 (Slang layout migration + perf-gate
// tooling, per the engine design spec's phasing) is expected to wire an
// automated comparison; until then, treat every number here as
// informational, not a gate.
//
// Run (after scripts/build.ps1, or the equivalent direct-configure
// build on Linux):
//   build-ninja/release/bin/spade_bench.exe --benchmark_format=json
//   build-ci/bin/spade_bench --benchmark_format=json
//
// Never invoke this (or any other Spade binary) from a backgrounded shell on
// this box -- see scripts/build.ps1's header and the plan's global
// constraints. Unlike spade_tests, spade_bench is a manual tool: run it in
// the foreground and read its own output directly.

#include <benchmark/benchmark.h>

#include "core/math_ops.hpp"

namespace {

// Throughput of the exp-map orientation update (spade::math::
// integrate_orientation), the hot inner loop of the engine's Integrate pass
// (see engine/core/math_ops.hpp's header comment: called once per body per
// substep). omega/dt are fixed at a physically-representative, non-trivial
// value so every iteration takes the std::cos/std::sin branch, not the
// tiny-angle Taylor-series shortcut near omega*dt ~= 0 -- the branch a
// non-degenerate simulation frame actually executes almost always.
void BM_IntegrateOrientation(benchmark::State& state) {
    glm::quat q(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::vec3 omega(0.3f, 0.5f, -0.2f);  // rad/s, comfortably above the 1e-6 tiny-angle cutoff
    const float dt = 1.0f / 200.0f;            // 200 Hz supervisor tick (foundation spec errata)

    for (auto _ : state) {
        q = spade::math::integrate_orientation(q, omega, dt);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(BM_IntegrateOrientation);

}  // namespace

BENCHMARK_MAIN();
