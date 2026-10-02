# Test/Docs — status

**The only place that describes what exists today in this realm.** Checked against the tree at `df33f09` (2026-10-02) unless a row says otherwise. Every count carries its tree and commit (`TD-8`).

## Baseline (restructure plan R5)

| Preset | Tree | Source | Total | Passed | Skipped | Failed | Run |
|---|---|---|---|---|---|---|---|
| release | `build-ninja/release` (main tree) | `df33f09` | **921** | **919** | **2** | **0** | 2026-10-02, `scripts\test.ps1 -Preset release`, 174.4 s |
| debug | `build-ninja/debug` (main tree) | `df33f09` | **921** | **919** | **2** | **0** | 2026-10-02, the same `ctest` arguments in three `-I` legs (310 + 310 + 301, checked with `ctest -N`), 264.1 s |

- **Both skips are by design.** `Fp32Exp.FullDomainSweepEveryFloatArgument` runs only with `SPADE_FULL_EXP_SWEEP=1`. `SlangLayouts.DeliberatelyUnboundArraysHaveNoBinding` skips while no array is exempt from binding.
- **GPU:** 65 tests carry `gpu`, and all 65 ran and passed on both presets on this box's device.
- **The two presets register the same 921 test names** (sorted lists compared byte for byte).
- **How it got here from the first measurement** (release, `7637f11`, 897 / 895 / 2 / 0, with a sibling `../KAT` checkout present): KAT-reach removed 31 cases that read KAT's content and added 3 `AgreementProbe`; then the drone box added 33 (`SandboxDrone*`), the rotor wake 14 (`RotorWake`), Core's medium sampling and defect fixes 4, and builder winding 1. Measured by name, not by subtraction. The suite no longer reads anything outside this repository, so the total no longer depends on the machine.
- **Build times** at `-ParallelLevel 1`: release from a fresh checkout, including dependency fetch, 1006.5 s; release incrementally from `2237048` to `df33f09`, 493.4 s; debug from scratch, 649.9 s including configure.

## Specified vs built

| Ruling | Built | Evidence and gaps |
|---|---|---|
| `engine D11` | yes, except own CI | GTest/CTest, the replay spine (`replay.hpp`), the parity harness (`parity.hpp`) and the bench (`spade_bench`, `baselines.json`) all exist. There has been no hosted CI since 2026-09-18 (`.github/` is absent) |
| `engine D12` (toolchain) | yes | MSVC + Ninja presets; CMake 4.2.0 on this box against a 3.28 floor |
| `SL7` (guard) | yes | `TransferRegister.*`, 4 tests; 14 rows pinned; 1 row open (SPH fluid) |
| `SL15b` (test half) | partly | `spade_sandbox --headless` exists; it is not part of the gate |
| `SL18` | partly | Obligation 5 is met. Obligations 6–9 wait on the editor (Interface), 4 on SPH (Physics), and 3's cross-backend half is checked on a developer machine only |
| SPIR-V rules | yes | `test_slang_layouts.cpp`: 10 tests (`SlangLayouts.*` 6, `SlangSpirv.*` 4), none device-gated |
| `TD-1` golden governance | partly | 5 scenarios, 7 worlds and 3 render manifests, each with provenance. The required cross-platform check has no second toolchain since the split (below) |
| `TD-6` device gating | yes | Every `Gpu*` suite gets `gpu`; 65 at `df33f09` |
| `TD-7` the gate | yes | `scripts\test.ps1`; no hosted CI |

## Needs a user decision

1. **The second toolchain.** Golden discipline used to require a gcc-13 check before a regeneration was final. That check ran on hosted CI and later on KAT's Linux docker leg, and neither exists now. Options: restore a Linux leg (WSL or a container on this box), or record that goldens are pinned for MSVC only until one exists. Today the gate cannot see a Linux-only divergence.
2. **Hosted CI.** `engine D11` says "own CI legs". Restore it, or amend `D11` to the one-machine gate.

## Debt

- **The consumer smoke** (`tests/consumer/`) runs nowhere (`../backlog.md`, with Interface).
- **Test discovery writes into the source root.** `gtest_discover_tests()` runs in the tests' `WORKING_DIRECTORY` (the source root), so CMake 4.x leaves `cmake_test_discovery_<hash>.json` there. It is ignored (`.gitignore`); moving the working directory would change every test's CWD, and is not worth that today.
