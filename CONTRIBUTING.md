# Contributing to Spade

Spade's value is that a run is **reproducible** and a parity claim is **checkable**. Almost every
rule below exists to protect one of those two properties, and most of them cost somebody a full
investigation at least once.

## Build and test

All three scripts are PowerShell and take `-Preset debug` or `-Preset release` (default
`release`).

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build.ps1
powershell -ExecutionPolicy Bypass -File scripts\test.ps1
powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -Scene hover
```

⛔ **Run builds and tests in the FOREGROUND.** A backgrounded build gets killed on the reference
box — not by the overrun itself, but because overrunning moves the job into a background queue
that an idle-memory reaper collects. Size a leg to finish inside the window and it never enters
that queue.

⛔ **Never invoke `spade_tests.exe` directly.** `ctest` is what turns each `TEST()` into one
accounted-for, filterable, timed case. `scripts\test.ps1` drives `ctest -L spade`.

**`scripts\test.ps1` is the gate.** There is no hosted CI. A change is green when that script is
green on both presets.

> ⚠ **One coverage gap is owed and is recorded here rather than left implicit.** Until 2026-09-28
> an out-of-tree consumer smoke ran in Kat's gate: it configured `tests/consumer/` against an
> **installed** Spade and linked it. That proves something `spade_tests` cannot — that the
> installed tree is consumable by a project that is not this one, which can fail while every
> in-tree test passes (it did once, on a `yaml-cpp` link closure that only a static-archive
> consumer pulls). `tests/consumer/` is still here and still correct; **the leg that runs it is
> not, and rebuilding it in this repo's own gate is owed work.**

## Determinism — the rules with no exceptions

**The op order of each physics pass IS the CPU/GPU parity contract** (`D1`/`D11`). Reordering
arithmetic inside a pass is a behaviour change even when it is algebraically identical, because
fp32 is not associative. Non-MSVC builds compile with `-ffp-contract=off` precisely so the
optimizer cannot fuse it behind your back.

**fp32 only** for state and math. No doubles in the state arrays.

**Seeded RNG through `rng::Stream`,** with domain-tag discipline. Never
`std::random_device`, never an unseeded default.

**No wall-clock reads** anywhere in `engine/` outside `tools/` and bench timers. A simulation that
can observe real time is not reproducible.

### Regenerating a golden digest has a price, and you pay it in writing

`tests/golden/scenarios/*.scenario.yaml` each carry their own `expected_digest`. If a change moves
a digest:

1. **Understand why it moved before you regenerate it.** A moved digest is either a deliberate
   physics change or a bug. Those look identical in the diff.
2. **Update that scenario's own `PROVENANCE OF expected_digest` block** to name what moved and
   why. A drive-by regeneration is not acceptable — the test quotes this discipline back at a
   diverging run for a reason.
3. ⛔ **The GPU is never a golden source.** The corpus is CPU-golden. Device-dependent parity is
   compared live, every run, and a GPU result never becomes the recorded expectation.

## Structural rules the build enforces

- **Dependency arrows point strictly downward**: `sim` → `objects`/`vehicles`/`physics` → `world`
  → `state` → `core`. Nothing below a layer knows the layer above it exists.
- **No exceptions across a module boundary.** Fallible calls return `Result<T>`
  (`std::expected`-based). A function that can fail says so in its type.
- **Nothing outside `engine/compute/` includes a Vulkan header.** Machine-checked by
  `test_slang_layouts.cpp`.
- **The state arrays are frozen at 18.** Derived GPU storage is a host↔device mirror and must not
  call `register_array` — adding one changes `kSnapshotVersion` and every parity band.
- **Shader parity profile**: no `OpFDiv`, no `sqrt`, and no GLSL.std.450 transcendental import on
  any parity-profile module. Use `fp32_math.slang`'s `log32`/`exp32`/`sin32`/`cos32`/`div32`.
  `spirv_scan.hpp` enforces rules P1–P5 and E1/E2 on every compiled variant.
- **`docs/v1-transfer-register.md` is normative and machine-checked.** A row's disposition must be
  one of `transferred`, `to-transfer`, `retired-with-reason`, `retired-to-sandbox`.
  **A silent drop is not a disposition.**

## v1 is frozen

`src/`, `include/`, `examples/` and `assets/` are the original v1 OpenGL engine. Do not develop
them. They remain because of the transfer register:

⛔ **v1 may not be quarantined while any register row is still open.** One is: the SPH fluid
solver. v1 is still the only implementation of it, so removing v1 would retire a live capability
rather than a superseded one.

## Documentation

- A file under `docs/design/superseded/` describes the world as of its own date. **Do not correct
  a stale claim inside one** — editing a historical record to agree with today destroys the
  evidence of what was believed when the decision was made.
- When you change behaviour, the document that asserts the old behaviour is part of the change.
  A finding is reported in the commit that fixes it.

## Reporting a measurement

Say which thing you have. **A clean compile proves syntax and types and nothing about behaviour**
— "compiled" and "green" are different claims, and a tally is worth nothing without the tree it
was measured in. A figure keeps the commit it was measured at, not the commit it is filed under.
