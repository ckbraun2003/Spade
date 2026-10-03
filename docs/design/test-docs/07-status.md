# Test/Docs — status

**The only place that describes what exists today in this realm.** Checked against master at `3f79529` (2026-10-03, pushed to origin) unless a row says otherwise. Nothing outside `docs/` has changed since the lead's batch gate at `3605ddf`, so that measurement describes this tree. Every count carries its tree and commit (`TD-8`).

## Baseline (restructure plan R5)

| Preset | Tree | Source | Total | Passed | Skipped | Failed | Run |
|---|---|---|---|---|---|---|---|
| release | `build-ninja/release` (main tree) | `df33f09` | **921** | **919** | **2** | **0** | 2026-10-02, `scripts\test.ps1 -Preset release`, 174.4 s |
| debug | `build-ninja/debug` (main tree) | `df33f09` | **921** | **919** | **2** | **0** | 2026-10-02, the same `ctest` arguments in three `-I` legs (310 + 310 + 301, checked with `ctest -N`), 264.1 s |
| release | `build-ninja/release` (main tree) | `88a3c8b` (the commit pushed to origin, plus docs) | **921** | **919** | **2** | **0** | 2026-10-02, lead, `scripts\test.ps1 -Preset release`, 161.3 s; 65 `gpu` ran |
| debug | `build-ninja/debug` (main tree) | `88a3c8b` | **921** | **919** | **2** | **0** | 2026-10-02, lead, `scripts\test.ps1 -Preset debug`, 271.4 s; 65 `gpu` ran |
| release | `build-ninja/release` (main tree) | `3605ddf` (round 2's batch gate) | **978** | **976** | **2** | **0** | 2026-10-03, lead, `scripts\test.ps1 -Preset release`, 215.5 s; 78 `gpu` ran |
| debug | `build-ninja/debug` (main tree) | `3605ddf` | **978** | **976** | **2** | **0** | 2026-10-03, lead, `scripts\test.ps1 -Preset debug`, 413.6 s; 78 `gpu` ran |

- **Both skips are by design.** `Fp32Exp.FullDomainSweepEveryFloatArgument` runs only with `SPADE_FULL_EXP_SWEEP=1`. `SlangLayouts.DeliberatelyUnboundArraysHaveNoBinding` skips while no array is exempt from binding.
- **GPU:** 78 tests carry `gpu` at `3605ddf`, and all 78 ran and passed on both presets on this box's device (`TD-13`).
- **The two presets register the same 978 test names** at `3605ddf` (sorted `ctest -N -L spade` lists compared byte for byte).
- **From 921 to 978, by name** (`88a3c8b` to `3605ddf`): 60 added, 3 removed.
  - Added by suite: `ModuleSchedule` 17, `GpuGlRenderer` 10, `RenderField` 6, `ModuleSimulation` 6, `ModuleSnapshot` 4, then 2 each for `StandardModules`, `SnapshotFormat`, `ScenarioFile`, `SandboxTargetSink` and `RenderShading`, and 1 each for `Schedule`, `ScenarioCorpus`, `GpuStateMirrorTest`, `GpuParityTest`, `GpuInvarianceTest`, `GnssReseed` and `GlRendererOptions`.
  - Removed: `ScenarioCorpus.IsExactlyTheFiveCommittedScenarios`, which became `...TheSix...`, and the two `Schedule.*SpecSectionThree*` tests, which became `Schedule.TheCompiledStandardSetFollowsTheGpuRecordersOrder` under the module API.
  - The 13 new `gpu` tests are `GpuGlRenderer` 10 plus one each in `GpuStateMirrorTest`, `GpuParityTest` and `GpuInvarianceTest`.
- **How it got here from the first measurement** (release, `7637f11`, 897 / 895 / 2 / 0, with a sibling `../KAT` checkout present): KAT-reach removed 31 cases that read KAT's content and added 3 `AgreementProbe`; then the drone box added 33 (`SandboxDrone*`), the rotor wake 14 (`RotorWake`), Core's medium sampling and defect fixes 4, and builder winding 1. Measured by name, not by subtraction. The suite no longer reads anything outside this repository, so the total no longer depends on the machine.
- **Build times** at `-ParallelLevel 1`: release from a fresh checkout, including dependency fetch, 1006.5 s; release incrementally from `2237048` to `df33f09`, 493.4 s; debug from scratch, 649.9 s including configure.

## Specified vs built

| Ruling | Built | Evidence and gaps |
|---|---|---|
| `engine D11` | yes, except own CI | GTest/CTest, the replay spine (`replay.hpp`), the parity harness (`parity.hpp`) and the bench (`spade_bench`, `baselines.json`) all exist. "Own CI legs" is amended by `TD-11` (self-run Docker legs), which is not built yet (`../backlog.md`) |
| `engine D12` (toolchain) | yes | MSVC + Ninja presets; CMake 4.2.0 on this box against a 3.28 floor |
| `SL7` (guard) | yes | `TransferRegister.*`, 4 tests; 14 rows pinned; 1 row open (SPH fluid) |
| `SL15b` (test half) | partly | `spade_sandbox --headless` exists; it is not part of the gate |
| `SL18` | partly | Obligation 5 is met. Obligations 6–9 wait on the editor (Interface), 4 on SPH (Physics), and 3's cross-backend half is checked on a developer machine only |
| SPIR-V rules | yes | `test_slang_layouts.cpp`: 10 tests (`SlangLayouts.*` 6, `SlangSpirv.*` 4), none device-gated |
| `TD-1` golden governance | partly | 6 scenarios, 7 worlds and 3 render manifests, each with provenance. `gnss_tumble` joined at `0ce4ff5` (`PHY-6`) on the existing `ballistic` world; its digest is provisional under `TD-12` until the Docker leg reproduces it. The 8 viewer trajectories (`tests/golden/viewer/`, `INT-4`, `0935c6f`) are goldens too, asserted once Interface's guard lands; the perishable captures beside them are an archive, not goldens (`tests/v1-baselines/`). The cross-check that makes a regeneration final is `TD-12`'s, not built yet |
| `TD-6` device gating | yes | Every `Gpu*` suite gets `gpu`; 78 at `3605ddf` |
| `TD-7` the gate | yes | `scripts\test.ps1` on both presets, on the development box only (`TD-11`) |
| `TD-11` Docker leg | no | Ruled 2026-10-02; no image, script or Dockerfile exists. First in the backlog's order (`../backlog.md`) |
| `TD-12` golden cross-check | no | Waits on `TD-11`'s leg. Until it runs, no regeneration can be final, which is why `RND-5` follows the leg. The gate cannot see a Linux-only divergence |
| `TD-13` GPU coverage | yes, as policy | All 78 `gpu` tests ran on both presets at `3605ddf`; none skipped. Nothing mechanical enforces it: `scripts\test.ps1` exits 0 with skips, so each report names any skipped `gpu` test (`02-build-and-gate.md`) |

## Ruled by the user (2026-10-02)

The two decisions this page held are ruled; the rulings live in `00-decisions.md`.

1. **The second toolchain** is the Docker gcc leg: a regenerated golden is final only once that leg reproduces it (`TD-12`).
2. **Hosted CI** is not restored. CI is self-run in Docker on this box, which amends `engine D11` (`TD-11`).

GPU coverage on a one-machine gate, the third open question, is `TD-13`. No user decision is open in this realm.

## Debt

- **The consumer smoke** (`tests/consumer/`) runs nowhere yet. The Docker leg is to run it with `SPADE_VULKAN` ON and OFF (`TD-11`; `../backlog.md`, with Interface).
- **Test discovery writes into the source root.** `gtest_discover_tests()` runs in the tests' `WORKING_DIRECTORY` (the source root), so CMake 4.x leaves `cmake_test_discovery_<hash>.json` there. It is ignored (`.gitignore`); moving the working directory would change every test's CWD, and is not worth that today.

## What's next

1. **The Docker leg** (`TD-11`, `TD-12`), which takes in the consumer smoke. One script builds a Linux/gcc image, runs `ctest -L spade -LE gpu` on release, and builds and runs `tests/consumer` against an installed prefix with `SPADE_VULKAN` ON and OFF. Core's untracked `tasks/core-offtree.ps1` is the recipe for the consumer half (MSVC-only, hard-coded paths). Nothing has compiled under gcc since the split, so the first run is likely to find portability failures in other realms' code; those go to their owners. A spec and plan come first, in `plans/`.
2. **GPU coverage** (`TD-13`) is ruled and written into `02-build-and-gate.md`. What remains is practice, not code: every gate report states how many `gpu` tests ran and names any skip, and the Docker leg's report states that it excluded them.
