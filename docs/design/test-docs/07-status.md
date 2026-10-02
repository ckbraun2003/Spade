# Test/Docs — status

**The only place that describes what exists today in this realm.** Checked against the tree at `186999e` (2026-10-01) unless a row says otherwise. Every count carries its tree and commit (`TD-8`).

## Baseline (restructure plan R5)

| Preset | Tree | Source | Total | Passed | Skipped | Failed | Notes |
|---|---|---|---|---|---|---|---|
| release | `build-ninja/release` | `7637f11` (configured at `2237048`; no non-doc change between) | **897** | **895** | **2** | **0** | 2026-10-01, `scripts\test.ps1 -Preset release`, 213.7 s |
| debug | `build-ninja/debug` | — | — | — | — | — | not yet run; waits for its build slot |

- **Both skips are by design.** `Fp32Exp.FullDomainSweepEveryFloatArgument` runs only with `SPADE_FULL_EXP_SWEEP=1`. `SlangLayouts.DeliberatelyUnboundArraysHaveNoBinding` skips while no array is exempt from binding.
- **GPU:** 63 tests carry `gpu`, and all 63 ran and passed on this box's device. None skipped.
- **The total depends on the machine.** A sibling `../KAT` checkout existed, so the 30 `ShippedWorlds/AgreementMatrix.*` cases and `AgreementFixture.BookmarksMatchTheShippedScenes` ran against KAT's content. Rendering's KAT-reach branch removes those 31 and adds 3 `AgreementProbe` cases, so the total after it merges is 869 plus whatever other branches add.
- **First build:** release, from a fresh checkout including dependency fetch, 1006.5 s at `-ParallelLevel 1`.

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
| `TD-6` device gating | yes | Every `Gpu*` suite gets `gpu`; 63 on this tree |
| `TD-7` the gate | yes | `scripts\test.ps1`; no hosted CI |

## Needs a user decision

1. **The second toolchain.** Golden discipline used to require a gcc-13 check before a regeneration was final. That check ran on hosted CI and later on KAT's Linux docker leg, and neither exists now. Options: restore a Linux leg (WSL or a container on this box), or record that goldens are pinned for MSVC only until one exists. Today the gate cannot see a Linux-only divergence.
2. **Hosted CI.** `engine D11` says "own CI legs". Restore it, or amend `D11` to the one-machine gate.

## Debt

- **The consumer smoke** (`tests/consumer/`) runs nowhere (`../backlog.md`, with Interface).
- **Stale counts and claims in the front door:**
  - `README.md` says 521 tests and 59 `gpu`; this tree has 897 and 63.
  - `README.md` cites `docs/design/01-charter.md` and `02-engine.md`, which moved.
  - `CONTRIBUTING.md` says "frozen at 18". Core measures 22 registered arrays (`../core/07-status.md`).
  - The README says `world_version` is "currently 1" in one place and 2 in another; 2 is right.
  - `CHANGELOG.md` says Spade is "developed inside the kat monorepo".
  
  Owed by R4 item 4.
- **KAT references** in `scripts/`, `CMakePresets.json`, `tests/CMakeLists.txt`, `tests/bench/baselines.json` (`_meta`), `cmake/SpadeSlang.cmake` and `scripts/pre-push-guard.sh`. Owed by R4 item 3 and the R2 comment sweep.
- **`T0` label.** Every test carries `T0`, a KAT tier name. Decide whether to keep it and define it locally (R4 item 3).
- **Stray file.** Test discovery writes `cmake_test_discovery_<hash>.json` into the source root. Point discovery's working directory into the build tree, or ignore the file.
- **The pre-push hook.** It is installed and matches the tracked copy apart from line endings. The tracked copy checks out as CRLF (`core.autocrlf=true`), so a reinstall must strip CRs (R4 item 3).
- **A link into a moved file.** `docs/v1-transfer-register.md` cites `docs/design/06-sandbox-and-v1.md`, which is now under `superseded/2026-09-consolidation/`.
- **The `tests/CMakeLists.txt` header** counts 67 GPU-file test sites, 61 of them device-gated, and still describes spade.yml-era CI. Recount it, or point it at `TD-6`.
