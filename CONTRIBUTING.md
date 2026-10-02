# Contributing to Spade

Start with `AGENTS.md`. It is short on purpose. This file adds the rules that protect what Spade is for: a run that is **reproducible** and a parity claim that is **checkable**. The design library (`docs/design/`, start at `INDEX.md`) holds the reasoning; the IDs below point into it.

## Build and test

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build.ps1 [-Preset debug|release]
powershell -ExecutionPolicy Bypass -File scripts\test.ps1  [-Preset debug|release]
```

- **The gate is `scripts\test.ps1`, green on both presets** (`TD-7`). There is no hosted CI.
- **Run builds and tests in the foreground.** A backgrounded job gets killed on the reference machine.
- **Never run `spade_tests.exe` directly.** ctest is what turns each `TEST()` into one counted, filterable, timed case.
- **Report a count with its build tree and commit, and name every skip** (`TD-8`). "Compiled" and "green" are different claims.
- **A GPU test lives in a `Gpu*` suite and starts with the device check** (`TD-6`). If you add or change one, run it on a device and say so in the commit; nothing downstream will.

Script and gate details: `docs/design/test-docs/02-build-and-gate.md`.

## Determinism

- **Op order inside a pass is part of its contract.** fp32 is not associative, so reordering arithmetic is a behaviour change even when it is algebraically identical. Non-MSVC builds use `-ffp-contract=off` so the optimiser cannot fuse it.
- **fp32 for state and math** unless a module declares otherwise as part of its grade (`docs/design/00-charter.md`).
- **Seed every random draw through `rng::Stream`** with its own domain tag. Never `std::random_device`, never an unseeded default.
- **No clock in the step path** (`L1`). Wall-clock reads belong in tools and bench timers only.
- **No libm transcendental on a path that feeds state or a digest**: use `core/fp32_math` (`TD-3`). On the GPU the SPIR-V rules enforce the same thing.

## Goldens and bands

- **Understand why a golden moved before you regenerate it** (`TD-1`). A moved digest is either a deliberate change or a bug, and the two look identical in a diff. The new value carries its provenance in the golden's own record.
- **The GPU is never a golden source** (`L4`). CPU↔GPU agreement is compared live, inside a measured band (`TD-2`).
- **A band is never widened to make a failure pass** without a root cause. If a band cannot be met, the module's grade changes; the band does not.

## Structure the build enforces

- **Dependencies point strictly downward** (`sim` → `objects`/`vehicles`/`physics` → `world` → `state` → `core`).
- **No exception crosses a module boundary.** A fallible call returns `Result<T>`.
- **Nothing outside `engine/compute/` includes a Vulkan header** (checked by `test_slang_layouts.cpp`).
- **Registered state is append-only, and the snapshot walk is pinned by test.** Changing what is registered is a Core decision (`docs/design/core/02-state-and-snapshot.md`).
- **Parity kernels obey the SPIR-V rules** `P1`–`P5`, and `fp32_math` modules also `E1`/`E2` (`docs/design/test-docs/00-decisions.md`).
- **`docs/v1-transfer-register.md` is machine-checked.** Every v1 system has one of four dispositions, and v1 stays in the tree while any row is open.

## Docs and commits

- **A behaviour change carries its doc change** in the same commit.
- **Never edit the contents of `docs/design/superseded/`.** It records what was believed and when.
- **Cite documents by section, never by line number.**
- **Commit with explicit paths** when other sessions share the tree, and **never push** without the owner's word. The pre-push guard (`scripts/pre-push-guard.sh`) enforces the second; install it in each clone.

Documentation rules: `docs/design/test-docs/03-documentation.md`.
