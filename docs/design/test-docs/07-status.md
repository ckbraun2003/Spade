# Test/Docs — status

**The only place that describes what exists today in this realm.** Checked against master at `7badaf1` (2026-10-03) plus this realm's branches unless a row says otherwise. The Windows counts below are from the lead's batch gate at `3605ddf`; the Docker leg's are from `fe4934a`, which contains `7badaf1`. Every count carries its tree and commit (`TD-8`).

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

## The Docker leg's first run (`TD-11`, `TD-12`)

`scripts\docker-leg.ps1` in the `spade-docker-leg:ed5552f460d7` image: gcc-13 13.3.0, CMake 3.28.3, Release, `-j1`, `--memory 3g`. The commit is `fe4934a`, a local integration commit that was never pushed: master `7badaf1` plus `test-docs/docker-leg`, `test-docs/viewer-canary` and `interface/consumer-smoke`. Run on 2026-10-03.

| Run | configure | build | test | agreement | consumer ON | consumer OFF | Total | Peak memory |
|---|---|---|---|---|---|---|---|---|
| `fe4934a` as committed | PASS, 5 s | **FAIL**, 1031 s: 1 of 263 steps | blocked | blocked | blocked | PASS, 236 s | 1272 s | 845 MB |
| `fe4934a` + braces at `tests/test_sandbox_drone.cpp:76`, in the container only | PASS, 3 s | PASS, 13 s | **908 run: 906 passed, 2 skipped, 0 failed**, 55 s | no matrix in this commit | PASS, 62 s | PASS, 45 s | 178 s | 362 MB |

- **The one gcc error:** `tests/test_sandbox_drone.cpp:76:16` `-Werror=dangling-else`, a gtest `EXPECT_LT` inside an unbraced `if`. It is Interface's, routed by the lead. Every other translation unit compiles under `-Wall -Wextra -Wpedantic -Werror`.
- **`TD-12`, first cross-check since the split:**
  - all 6 scenario digests reproduce on gcc (`Determinism.DigestsMatchTheCommittedGoldenCorpus`), `gnss_tumble` included;
  - the render goldens (`RasterGolden` ×5, `TessellateGolden`, `CsgMeshGolden`) pass;
  - all 8 viewer trajectories pass at full length (`SPADE_FULL_VIEWER_TRAJECTORIES=1`; shower takes 28.06 s, the other seven under 0.4 s).
  
  The brace fix touches one test file that feeds no digest.
- **Skips and `gpu`:** the two skips are the same by-design pair as on Windows. 987 names are registered, and 79 `gpu` tests are excluded with `-LE gpu` (`TD-13`).
- **Names against Windows:** the 987 on Linux differ from the main tree's Windows list (978, built at `3605ddf`) by 11 added and 2 removed. All 13 come from commits after `3605ddf`: `0835e32` (the viewer guard), `3925882` (render no-data) and `7aa3060` (the heatmap layer). No test registers on one platform only.
- **The canary narrowing** (`test-docs/viewer-canary`, `f9942f7`) passes. Its mutation control went red, then green: tokens planted in the container's `scenes.cpp` failed all four scan tests at the planted lines (`chrono` at :870, a v1 include at :871, `std::sin(` at :872), and they passed again after the re-sync.
- **Under memory pressure:** host memory fell to about 480 MB free, and the harness stopped the background watcher. The detached container carried on, and `-Follow` collected the result. Host memory, not the container's 3 GB, limits this box, so the leg stays at `-j1`. C: had 12.4 GB free before and after.
- **Earlier one-off costs** (slot 5, 2026-10-02): the image build took 228 s (762 MB), and the first configure, including the dependency fetch, took 211 s.

## Specified vs built

| Ruling | Built | Evidence and gaps |
|---|---|---|
| `engine D11` | yes | GTest/CTest, the replay spine (`replay.hpp`), the parity harness (`parity.hpp`) and the bench (`spade_bench`, `baselines.json`) all exist. "Own CI legs" is amended by `TD-11`: the self-run Docker leg (below) |
| `engine D12` (toolchain) | yes | MSVC + Ninja presets; CMake 4.2.0 on this box against a 3.28 floor |
| `SL7` (guard) | yes | `TransferRegister.*`, 4 tests; 14 rows pinned; 1 row open (SPH fluid) |
| `SL15b` (test half) | partly | `spade_sandbox --headless` exists; it is not part of the gate |
| `SL18` | partly | Obligation 5 is met. Obligations 6–9 wait on the editor (Interface), 4 on SPH (Physics), and 3's cross-backend half is checked on a developer machine only |
| SPIR-V rules | yes | `test_slang_layouts.cpp`: 10 tests (`SlangLayouts.*` 6, `SlangSpirv.*` 4), none device-gated |
| `TD-1` golden governance | partly | 6 scenarios, 7 worlds and 3 render manifests, each with provenance. `gnss_tumble` joined at `0ce4ff5` (`PHY-6`) on the existing `ballistic` world, and the Docker leg reproduced its digest at `fe4934a`, so it is no longer provisional. The 8 viewer trajectories (`tests/golden/viewer/`, `INT-4`, `0935c6f`) are goldens too, asserted by `Viewer/ViewerTrajectory.*` (`0835e32`); the perishable captures beside them are an archive, not goldens (`tests/v1-baselines/`). A regeneration is final once the leg reproduces it (`TD-12`) |
| `TD-6` device gating | yes | Every `Gpu*` suite gets `gpu`; 78 at `3605ddf` on Windows, 79 registered at `fe4934a` |
| `TD-7` the gate | yes | `scripts\test.ps1` on both presets, on the development box only (`TD-11`) |
| `TD-11` Docker leg | yes | `scripts\docker-leg.ps1`, `docker-leg.sh` and `docker-leg.Dockerfile`, with Interface's `scripts/consumer-smoke.sh`. It ran end to end at `fe4934a` (above). It runs when a slot is given, not on every commit |
| `TD-12` golden cross-check | yes | First run at `fe4934a`: 6 scenario digests, the render goldens and 8 viewer trajectories reproduce on gcc. The gate still cannot see a Linux-only divergence between leg runs |
| `TD-13` GPU coverage | yes, as policy | All 78 `gpu` tests ran on both presets at `3605ddf`; none skipped. The Docker leg excludes them with `-LE gpu` (79 at `fe4934a`). Nothing mechanical enforces it: `scripts\test.ps1` exits 0 with skips, so each report names any skipped `gpu` test (`02-build-and-gate.md`) |

## Ruled by the user (2026-10-02)

The two decisions this page held are ruled; the rulings live in `00-decisions.md`.

1. **The second toolchain** is the Docker gcc leg: a regenerated golden is final only once that leg reproduces it (`TD-12`).
2. **Hosted CI** is not restored. CI is self-run in Docker on this box, which amends `engine D11` (`TD-11`).

GPU coverage on a one-machine gate, the third open question, is `TD-13`. No user decision is open in this realm.

## Debt

- **Test discovery writes into the source root.** `gtest_discover_tests()` runs in the tests' `WORKING_DIRECTORY` (the source root), so CMake 4.x leaves `cmake_test_discovery_<hash>.json` there. It is ignored (`.gitignore`); moving the working directory would change every test's CWD, and is not worth that today.

## What's next

1. **Put the leg to work.** Rendering's `RND-5` regeneration is final only once the leg reproduces it. Rendering's agreement bands need its `gcc-release` attestation, which the leg's `agreement` part collects.
2. **A green leg on the committed tree,** once Interface's brace fix for `test_sandbox_drone.cpp:76` lands.
3. **GPU coverage** (`TD-13`) is ruled and written into `02-build-and-gate.md`. What remains is practice, not code: every gate report states how many `gpu` tests ran and names any skip, and the Docker leg's report states that it excluded them.
4. **The bench baselines' next re-seed** names Core's module-API stage 2 commit in `_meta`. That commit splits the rotor/drag and IMU/GNSS timing brackets, so `gpu_force_elements_ns` and `gpu_sensor_synthesis_ns` each include one extra timestamp mark (`TD-8`).
