# Test/Docs — verification

**Owner:** Test/Docs owns the harness, golden governance, the parity harness, the SPIR-V scanner and the bench. Each realm writes its own tests and owns its own bands and goldens' content (`../physics/04-verification.md`, `../rendering/03-verification.md`). **Normative.** Rulings cited here live in `00-decisions.md`.

## The suite

- **One executable, one ctest case per test.** `spade_tests` (GoogleTest) is registered by `gtest_discover_tests` in `tests/CMakeLists.txt`, so every `TEST()` is its own ctest case: filterable, timed and counted. Run it through `scripts\test.ps1`, never directly (`TD-7`).
- **Labels** (`TD-10`). Every test carries `spade`, and a test in a `Gpu*` suite also gets `gpu` (`tests/AppendSpadeLabels.cmake`). Select or exclude with `ctest -L gpu` / `-LE gpu`.
- **Timeouts.** 60 s per test. Overrides are per test and carry their reason beside them: the opt-in full `exp` sweep (900 s), the shower-pile chaos control (180 s), `Gpu*` (180 s).
- **Layout.** One `test_<subject>.cpp` per concern. Test support lives in `engine/testing/` (`replay.hpp`, `scenario_file.hpp`, `parity.hpp`, `spirv_scan.hpp`): header-only, compiled into the tests, never installed.
- **Device-gated tests** follow `TD-6` and `TD-13`. On the development box they run as part of the gate, and a skip there is named and does not count toward green. On a machine without a device they skip, and the Docker leg excludes them with `-LE gpu` (`TD-11`).

**What a determinism test cannot see.** Two runs in one process read the same uninitialised memory and agree. A NaN is perfectly reproducible. Determinism proves reproducibility, not correctness, so a determinism test is never the only test of a pass.

## Grades and how each is verified

The charter's grades (`L3`, `L4`) set what a module's tests must show.

| Grade | Verified by |
|---|---|
| reference | a CPU implementation, a golden result (bit-identical per platform), and the module's own unit tests |
| banded | a live comparison against the reference on every run, inside a band pinned per `TD-2` and `TD-14` |
| best-effort | the module's unit tests; no numeric claim |

**Today** nothing declares a grade; the engine model is not built. The KAT-era equivalents stand in: the CPU golden corpus is reference grade, and CPU↔GPU parity is banded grade.

## Goldens

- **Scenario corpus.** `tests/golden/scenarios/*.scenario.yaml`, each naming a world in `tests/golden/worlds/`, an input script, a step count and its own `expected_digest` with a provenance block. Loaded by `scenario_file.hpp`. The digest is an FNV-1a fold over the state registry's walk (`replay.hpp`). Membership is pinned (`ScenarioCorpus.*`), so a scenario cannot drop out unnoticed.
- **Render goldens.** `tests/golden/render/{frames,csg,tessellation}/manifest.json`: SHA-256 pins with changelog keys. Their content is Rendering's.
- **Regenerating** follows `TD-1`, and a regeneration is final only once the Docker gcc leg reproduces it (`TD-12`; `02-build-and-gate.md`). The CPU is the only golden source (`L4`).

## Parity harness

`engine/testing/parity.hpp` compares the CPU and GPU paths per quantity and per element, at the run's final tick, against a table of bands (`TD-14`). An element passes when |gpu − cpu| ≤ abs + rel × |cpu|, and a NaN fails. The harness reports two maxima per quantity: `A`, the largest |gpu − cpu| where |cpu| < `s_q`, and `R`, the largest relative error where |cpu| ≥ `s_q` (`s_q` = 1e-3 in the quantity's SI unit). Element bands hold up to a scenario's horizon `T_p`, where one ulp of input, amplified by the CPU alone, stays under 1e-5 of the scene's scale (`ParityChaos.OneUlpControlAtEachScenariosHorizon`). Past it:
- **invariants** are exact on both paths: the same slots live, no NaN or infinity, no body centre inside a solid;
- **statistics** per world (centre of mass, mean and RMS speed, translational kinetic energy) are banded at 4× the largest deviation of the same statistic over eight one-ulp-perturbed CPU runs, never tighter than the floor (5e-10 + 5e-7 × |cpu|).

Each band carries its record (below), and the module's realm owns it. Two host-only guards keep the question asked even without a device: `ParityCorpus.EveryCorpusScenarioIsInTheParitySet` (membership) and the CPU halves of the invariance tests. `workgroup_size` is a live knob, and its invariance sweep has been shown to fail under a deliberately size-dependent reduction (`engine A7`, Core).

### Band records (`L4`)

Every pinned band carries its record in `parity.hpp`, at three levels, so nothing repeats per row. Copy the numbers from the table the parity test prints, which carries the device line in its header; never retype a device by hand.

- **Device**, once per device of record, above the bands. It holds:
  - the `compute::describe()` line, verbatim: name, driver and `driverVersion`, Vulkan version, float controls;
  - `vendorID` and `deviceID`;
  - the build preset, the commit and the date;
  - a short tag the rows cite, such as `rtx3060ti-572.83`.
  A device that stops measuring keeps its block, marked history.
- **Scenario**, once per scenario block. It holds:
  - steps × substeps and the horizon in seconds;
  - the one-ulp CPU control at that horizon, against the scene's scale (`T_p` holds, or element bands give way);
  - the method: "final tick, every element of every world, `A`/`R` split at `s_q` = 1e-3";
  - the margin: 4×, rounded up to one significant figure, the larger over devices of record.
- **Row**, beside each band, says which kind it is:
  - **measured**: per device tag, `A`, `R` and the band they give. Each half is 4× its measurement rounded up to one significant figure, and never tighter than the floor's half, so a measured `A` of 0 pins `abs` at 5e-10. `A` and `R` are written with enough significant figures to recompute the band;
  - **floor**: exactly equal on every device of record. `{5e-10, 5e-7}` is four ulps at `s_q` (4 × 2⁻³³ = 4.66e-10) and four ulps relative (4 × 2⁻²³ = 4.77e-7), each rounded up;
  - **structural**: `{0, 0}`, with the argument that no rounding operation touches it;
  - **cited**: the band of a signed row, such as `CORE-3`'s gaussian draw;
  - **bits**: integer lanes, `{0, 0}` by kind.
  An older device's numbers stay on the row, marked history, with the method they were measured under. The Iris rows were measured at the final tick under the disjunctive predicate; their provenance line's "whole run" was a misstatement (plan §2).
- **A widened band** names its cause (`TD-2`). For NVIDIA's division and square root, that is finding 4 of `../core/2026-10-04-nvidia-denorm-report.md`.

A row, with illustrative values:

```
// pos [m]  measured. rtx3060ti-572.83: A 2.1e-9, R 3.3e-7 -> {9e-9, 2e-6}
//          history, irisplus-2125 (final tick, disjunctive): max 4.1e-7 -> {2e-6, 1e-5}
```

## SPIR-V scanner

`engine/testing/spirv_scan.hpp` reads compiled SPIR-V as a word stream; it needs no SDK tool. It applies two profiles: **parity** kernels obey `SPIR-V rule P1`–`P5`, and **`fp32_math`** modules also obey `E1`/`E2`. `P3` was amended on 2026-10-05 and now holds that no module requests an fp32 denormal mode, so every device runs the kernels with its own default (`00-decisions.md`). Every compiled variant is scanned, and the module count comes from the build, so a new kernel cannot slip past unscanned. Run by `test_slang_layouts.cpp`, which needs no device.

## Bench

`spade_bench` (Google Benchmark) is **not** a ctest case. `tests/bench/` holds `bench_core`, `bench_sim` and `bench_render`, and `baselines.json` records reference runs. A timing is reported with its basis (`cpu_time` or `real_time`) and its between-batch spread (`TD-8`). The rate envelope is recorded, not gated; a miss is a design-review item, not a red build.

## Guards

- **Show it fail.** Every guard is demonstrated against the mutation it exists to catch (`SL18`). Then check what the mutation actually falsified, not just that something went red.
- **Expectations come from elsewhere** (`TD-4`).
- **Absence fails** (`TD-5`).
- **Two transcriptions of one authority** need a test that compares them, or they fork silently. Prefer deriving one from the other.

## Consumer smoke

`tests/consumer/` is an out-of-tree project that does `find_package(spade CONFIG)`, loads a world file and steps it. It proves the **installed** tree is usable, which no in-tree test can. The gate does not run it; the Docker leg does, with `SPADE_VULKAN` ON and OFF, through Interface's `scripts/consumer-smoke.sh` (`TD-11`, `02-build-and-gate.md`). Before the leg it ran only by hand: Core's run found restructure defect 3 (red at `b7616c1`, green at `6d40740`).
