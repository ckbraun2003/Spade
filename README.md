# Spade Engine

Spade is Kat's simulation engine: a fixed-step, deterministic physics core (the "v2" engine,
`engine/` + `tests/`) with a frozen v1 OpenGL renderer (`src/`, `include/`, `examples/`, `assets/`)
that v2's strangler viewer drives during the transition to a real (Vulkan) render backend.

> **Status (2026-08, v0.2.0):** Spade lives inside the Kat monorepo (`spade/`, subtree-merged with
> full history @ `1c8a133`) as the engine beneath Kat's sim stack. Its v2 direction -- fixed-step
> deterministic stepping, a CPU reference twin for every authoritative pass, headless/offscreen
> operation, seeded RNG, snapshots, native world-batching -- is pinned in
> `design-specs/kat-spade-engine-design.html` (local, untracked; approved v1.0, 2026-08-08). The S1-S4
> implementation plan (`.superpowers/plans/2026-08-08-spade-engine-s1-s4.md`) landed the CPU path of
> the charter's M1B bar -- see **M1B status** below. This README documents the engine **as it is
> today**; the design spec is the source of truth for where it is going.

## v2 quickstart

The commands below are written for the repo root (`C:\...\kat`) -- that leading `spade\` segment
is a relative path from there, not something the scripts resolve on their own. Running from inside
`spade\` itself: drop it (e.g. `scripts\build.ps1` instead of `spade\scripts\build.ps1`). All three
scripts are PowerShell, foreground-only (backgrounded builds/tests get killed on this box), and
each accepts `-Preset debug` or `-Preset release` (default `release`) -- the value maps internally
to `spade/CMakePresets.json`'s `msvc-ninja-debug` / `msvc-ninja-release` CMake preset names, but
also happens to be the literal build directory name (see Build, below); the two are related, not
identical.

### Build

```powershell
powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1
powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1 -Preset debug
powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1 -Preset release -Clean
```

Imports a vcvars64 MSVC environment (Ninja invokes `cl.exe` directly and needs it set up first),
configures the requested preset if it isn't already, and builds. Binaries land under
`spade/build-ninja/<debug|release>/bin/` -- the directory name is the `-Preset` VALUE you passed
(`debug` or `release`), not the longer CMake preset name (`msvc-ninja-debug` /
`msvc-ninja-release`) that value maps to internally.

### Test

```powershell
powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1
powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1 -Preset debug
powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1 -Filter IntegrateOrientation
```

Runs the v2 engine suite (`spade_tests`, gtest) through `ctest -L spade` against a build this preset
already produced -- **never invoke `spade_tests.exe` directly**; ctest is how each `TEST()` becomes
one accounted-for, filterable, timed case. **521 tests, both presets, green as of S6** (up from 437
at S5 close-out -- see "S6 status" below for the GPU-backend work that grew the suite; up from 349
before that -- the corpus grew across S5's world-file/scenario-file/reseed work; T9's hygiene batch
removed Task 1's `ecs/` scaffold and its 24-test suite (`test_ecs.cpp`, 459 -> 435) and added one
bit-portability canary (ticket C, 435 -> 436); the S5 final-review fix wave added the desc-ref
validation case (436 -> 437); S6's own hygiene pass (Task 10b) added two `scenario_file.hpp` loader
tests and one `world_file.hpp` op-node validation test, 518 -> 521). 59 of the 521 carry the
additional `gpu` ctest label (device-executing, `Gpu*` test-suite name prefix -- `ctest -L gpu` /
`-LE gpu` to select/exclude them): every one begins `if (!compute::vulkan_available())
GTEST_SKIP()`, so the count is 521 either way -- with a device they run for real, without one they
report skipped, never failed. A `SPADE_VULKAN=OFF` configure (no preset of its own; a manual
`-DSPADE_VULKAN=OFF` on top of either preset) drops the Vulkan backend from the build entirely and
registers 445 tests (437 + 5 + 3 exact -- the five S6 Task 2 tests plus Task 10b's three loader/
validation tests, none of them Vulkan-gated).

### Demo

```powershell
powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene hover
powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene flight -Preset debug
```

Builds (if needed) and launches one windowed demo scene. `-Scene` is one of:

| Scene | Renderer | What it shows |
|---|---|---|
| `drop`, `bounce`, `shower`, `gate` | `spade_viewer` (v2 physics through v1's GL stack) | Early v2 regression scenes: spheres onto a plane, a restitution ladder, a pile into an SDF bowl, a gate obstacle. |
| `hover` | `spade_viewer` | A `Quadrotor` spawned IN TRIM (`hover_command()`), holding a rock-steady hover -- the flight-scene control case, no turbulence. |
| `wind` | `spade_viewer` | The same hover under Dryden "moderate" turbulence -- altitude, not lean, is what visibly moves (see `tools/viewer/scenes.cpp`'s `scene_wind()` comment for why). |
| `flight` | `spade_viewer` | A scripted takeoff/hold/descend flight, starting below hover trim so the spin-up is visible. |
| `swarm` | `spade_viewer` | A 4-world set, one hovering quadrotor per world, stepped in ONE batched call (world-batching demo, D8). |
| `v1-fluid`, `v1-spheres`, `v1-cubes` | `Sandbox` (v1, unmodified) | v1's own GL engine, run as-is -- SPH fluid, brute-force spheres, boxes. Kept runnable as visual regression references while v2 progresses; `M` starts/stops the sim (paused on launch). |

`hover`/`wind`/`flight`/`swarm` are Task 20's flight demo + M1B verification scenes. Controls:
WASD + Space/Shift to fly the camera, `C` to toggle mouse capture, Esc or the window close button to
quit.

### World files & the scenario corpus (S5)

A world is no longer only a `WorldBuilder` call in C++: `world/world_file.hpp`'s
`save_world_file()`/`load_world_file()` round-trip a `WorldDesc` through a human-authorable YAML
file (schema version 1 -- see any file under `tests/golden/worlds/*.world.yaml` for the annotated
format, including the `world_version` upgrade key and the "9 significant digits, fp32 round-trips
exactly" float-spelling rule). `tests/consumer/main.cpp` is the worked, out-of-tree example: it
authors a world, saves it, reloads it, and only THEN builds a `WorldSetDesc` and steps a
`Simulation` from the reloaded copy -- proving the file format, not just the in-memory builder.

The determinism corpus (`tests/golden/scenarios/*.scenario.yaml`) is DATA in the same sense: each
scenario file names a world file by relative path, an input script, a step count, and its own
`expected_digest` -- loaded by `engine/testing/scenario_file.hpp` (test support, not shipped) rather
than hand-built as a C++ lambda. Regenerating a digest is a deliberate act with a cost: every
scenario file carries its OWN "PROVENANCE OF `expected_digest`" comment block naming what moved and
why (global-constraints discipline -- never a drive-by edit), and `test_determinism.cpp`'s golden
check quotes that discipline back at a diverging run.

`Simulation::reseed(scene_seed)` (S5 Task 4) rewrites every world's registered seed to a new scene
seed -- re-deriving the Dryden and sensor-noise streams that seed feeds -- without rebuilding the
`Simulation`: a `Result<void>` call that refuses over a pending structural queue and whose effect
survives a snapshot/restore round trip. `test_m1b_bar.cpp`'s `M1B.Reseed*` tests are Addendum A3's
second conformance case (determinism, discrimination from an un-reseeded continuation, and the
derivation pin against `replicate()`'s own formula).

## Module map

`engine/` (v2, C++23, fp32-only state & math, no exceptions across module boundaries,
`std::expected`-based `Result<T>`) -- one CMake target per subdirectory, dependency arrows point
strictly downward (`sim` -> `vehicles`/`physics` -> `world` -> `state` -> `core`; nothing below a
layer knows the layer above it exists):

| Directory | CMake target | What it is |
|---|---|---|
| `core/` | `spade::core` | Typed handles (`ids.hpp`), `Error`/`Result<T>` (`error.hpp`), the `Tick` step counter (`time.hpp`), math primitives (`math_ops.*` -- the exp-map orientation update), the seeded `rng::Stream` (`rng.hpp`, errata-R4 domain-tag discipline). No wall-clock reads anywhere in this tree except `tools/` and bench timers. |
| `state/` | `spade::state` | The state backbone: `layout.hpp`'s shared POD structs with `static_assert`ed std430 offsets (the Slang layout single-source, until S6), the world-partitioned `ArenaSet` + `StateRegistry` (`arenas.*`, `registry.*`), and the versioned snapshot/restore blob + file IO (`snapshot.*`) -- the engine's replay/determinism contract. |
| `world/` | `spade::world` | Analytic SDF scene programs (`sdf.*`), the `WorldBuilder` fluent API + its `WorldDesc` product (`builder.*`), the Dryden turbulence filter (`medium.*`), and the YAML world-file round trip (`world_file.*` -- S5 Task 5: `save_world_file()`/`load_world_file()`, schema version 1, see the quickstart's "World files & the scenario corpus" section). |
| `physics/` | `spade::physics` | The per-substep dynamics passes: `Integrate` (symplectic Euler + specific-force capture, `integrator.*`), `CollisionStatic` (sphere-proxy vs SDF, `contacts.*`), `CollisionDynamic` (world-batched sorted-grid broad phase, `grid.*`), and the `ForceElements` pass's drag law (`forces.*`). Each pass's op order is the CPU/GPU parity contract (D1/D11) -- compiled with `-ffp-contract=off` on non-MSVC so the optimizer cannot silently fuse it. |
| `sim/` | `spade::sim` | The fixed eight-pass substep **schedule** as data (`physics/schedule.*` -- compiles here, not into `spade_physics`, because only `Simulation` runs it), and `Simulation`/`WorldSet` (`sim/*`): arenas, tick, the substep loop, the structural queue, spawn/despawn, snapshot/restore, model registration. This is the engine's one public entry point. |
| `sensors/` | *(compiles into `spade::sim`)* | IMU synthesis (`imu.*`: mount-frame specific force + angular velocity, tick-stamped) and the sensor output ring buffers (`rings.hpp` -- editor tech spec TA5's sensor-poll convention, made executable). |
| `vehicles/` | `spade::vehicles` | The model-type layer (design spec section 6): `RotorElement` (thrust/torque curves, RPM lag, momentum-theory inflow, SDF ground effect -- `rotor.*`), the `ModelType` registry (`model_type.*`), and `Quadrotor` (`quadrotor.*`). "Nothing vehicle-specific below here" -- the dependency arrow points down into `physics`/`world`/`state`, never back. |
| `testing/` | *(header-only, test support only)* | `replay.hpp`: the determinism digest (`state_digest()`, an incremental FNV-1a fold over the state registry's walk order) and the scenario replay harness. `scenario_file.hpp` (S5 Task 7): the scenario-file loader -- the golden corpus is data, see the quickstart section above. Both included by `spade/tests/` only -- absent from every install/export rule, not something an out-of-tree consumer has any business with. |
| `tools/` | `spade_viewer` (`SPADE_BUILD_V1`-gated) | The strangler viewer: v1's frozen OpenGL renderer fed by v2 `Simulation` state, one demo scene at a time (`viewer/bridge.*`, `viewer/scenes.cpp`, `viewer/main.cpp`). The one place in `engine/` allowed a wall-clock read (frame pacing) and the one v2 code that includes v1 headers. Task 6 added a `-Backend cpu\|vulkan` switch (`demo.ps1`), so the same scenes can render off either backend's `Simulation`. |
| `compute/` | `spade::compute` (`SPADE_VULKAN`-gated, default `ON`) | S6's Vulkan/Slang GPU backend: `BackendDesc`/`BackendKind` (`backend.hpp`) select it at `Simulation::create()`; `vulkan/context.*` owns the instance/device/queue (device-capability checks, incl. `shaderDenormPreserveFloat32`); `vulkan/state_mirror.*` is the host<->device buffer mirror (derived storage only -- no `register_array` call, per the frozen-18-array global constraint); `vulkan/step_recorder.*` records the whole per-substep dispatch chain ONCE and resubmits it verbatim per step; `vulkan/timestamps.*` is the optional per-pass GPU timing S6 Task 10 added. `layout_check.cpp` + the generated `bindings.gen.hpp`/layout asserts are the single-sourced C++<->Slang struct mirror (`cmake/SpadeSlang.cmake`'s codegen). Nothing outside this directory includes a Vulkan header (spec §2's dependency rule, machine-checked by `test_slang_layouts.cpp`). |
| `shaders/` | Slang sources, compiled to embedded SPIR-V (no install rule) | The GPU port of `physics/`'s per-substep passes, one `.slang` kernel per schedule slot that has one (`kernels/integrate.slang`, `forces_drag.slang`, `collision_static.slang`; `sdf_eval.slang`/`shared/*.slang` are shared modules, not kernels) plus `compute/grid_entry.hpp`'s sorted-grid broad phase (`grid_build`/`grid_sort`/`collision_dynamic` in the CollisionDynamic chain) and `fp32_math.slang`/`fp32_math_probe.slang` (the `log32`/`exp32`/`sin32`/`cos32`/`div32` Slang port -- no `OpFDiv`, no `sqrt`, no GLSL.std.450 transcendental import on any parity-profile module, `spirv_scan.hpp`'s scanner rules P1-P5 + E1/E2, enforced on every compiled variant). Every kernel compiles once per `compute::kSupportedWorkgroupSizes` entry (`{32, 64, 128}`, S6 Task 9b) -- the `workgroup_size` backend knob is live, not a plumbing-only lever. |

`spade/tests/` (`spade_tests`, gtest, ctest labels `spade` + `T0`, plus `gpu` on every device-executing
`Gpu*` suite -- see the Test section above) mirrors this module list one `test_*.cpp` per concern,
plus `test_m1b_bar.cpp` (the M1B charter bar as executable asserts), `test_gpu_parity.cpp` (S6:
CPU<->GPU correctness -- see "S6 status" below), and the golden determinism corpus
(`tests/golden/scenarios/*.scenario.yaml`, five scenarios: `ballistic`, `bounce`, `shower`,
`two_world_isolation`, `quad_hover` -- each a committed data file naming its own world file
(`tests/golden/worlds/*.world.yaml`) and carrying its own `expected_digest` with a provenance
block, loaded by `engine/testing/scenario_file.hpp`); this corpus is CPU-golden only -- the GPU is
never a golden source, device-dependent parity is compared live, every run. `spade/tests/bench/`
(`spade_bench`, google-benchmark, **not**
ctest -- see its own header comments) holds `bench_core.cpp` (math primitive throughput) and
`bench_sim.cpp` (`Simulation::step()` throughput sweeps, CPU and GPU families); `baselines.json`
records reference runs, including S6 Task 10's per-pass GPU timing on this box's Intel Iris Plus
(the program's one correctness/dev-GPU device).

## M1B status

The charter's M1B bar -- fixed-step, headless, seeded determinism, snapshot/restore, 6-DOF +
Quadrotor + IMU -- is met on the **CPU path** (the charter sequences the GPU-path rows,
snapshot-GPU and instance-isolation-GPU, to S6; M1B is CPU by design, not by omission).
`test_m1b_bar.cpp`'s five charter-bullet tests, its summary table (a sixth TEST that re-derives and
prints all five results -- not a seventh charter bullet), and both Addendum A3 conformance cases
(`ResetPreservesRosterRoundTrip` and the `M1B.Reseed*` suite, see below) are all part of the
current `spade_tests` suite (521 tests as of S6, up from 437 at S5 close-out -- see "S6 status"
below) -- both presets green.

- **Version**: Spade is at **0.2.0** as of this claim (charter §5's version promise, Addendum
  A11(i)) -- this is Spade-the-project's own release number and is separate from the snapshot
  format's own version (`state/snapshot.hpp`'s `kSnapshotVersion`, currently `1`, a real symbol
  today) and from the world-file format's own version (`world_file.hpp`'s `world_version` key,
  currently `1` -- S5 Task 5 gave it a real version once the format itself existed). Neither is
  touched by this bump.
- **Rate envelope** (design spec §12): both CPU-measurable targets are **HIT** on this box
  (`spade/tests/bench/baselines.json` has the full run) -- a single world's quadrotor+IMU scene
  measures ~296,000 substeps/sec against a >=10,000 floor, and the 64-world batched set measures
  ~176,000 world-substeps/sec against a >=64,000 (64 worlds x 1 kHz) floor. The spec's batched
  target is phrased "on the dev GPU"; this is the CPU reference twin, so it is a strong signal, not
  the literal S6 measurement.
- **Addendum A3 conformance cases**: 2/2 written. `ResetPreservesRosterRoundTrip`
  (`test_m1b_bar.cpp`) covers the reset-preserves-roster case with today's engine surface
  (`restore()` into an earlier snapshot on the same `Simulation` IS a reset). The reseed
  determinism case (S5 Task 4, `M1B.Reseed*`) needed new engine surface --
  `Simulation::reseed(scene_seed)`, which did not exist during close-out -- and is now built and
  tested: determinism across two identically reseeded runs, discrimination from an un-reseeded
  continuation, the derivation pin against `replicate()`'s own formula, refusal over a pending
  structural queue, and survival across a snapshot/restore round trip.
- **What is NOT M1B**: the Vulkan/Slang GPU backend and real rendering (S6+), Linux GPU support
  (rides the Vulkan backend). The YAML world file and the `config`/`dt_ns`/`substeps` gap in
  `restore()`'s replay contract -- both listed here as open items at close-out -- are CLOSED as of
  S5 (world files: Task 5, `world_file.hpp`; the restore config gap:
  `test_determinism.cpp`'s `RestoreConfigCheck` suite, which rejects a blob restored under a
  different `dt_ns`/`substeps`/`config_hash`).

## S5 exit record (2026-08-12)

S5's exit bar (design spec, phase S5; plan `2026-08-11-spade-engine-s5`) is met on three legs,
each carried by committed artifacts rather than claims:

- **The corpus is data.** Five `spade/tests/golden/scenarios/*.scenario.yaml` + seven byte-pinned
  `worlds/*.world.yaml`; the S1-S4 builder lambdas are retired, with
  `GoldenCorpus.TheDataScenariosReproduceTheRetiredBuilderCorpus` (`test_determinism.cpp`) as the
  permanent equivalence record -- it hard-codes the four builder-era digests as a second source and
  checks both the files and the runs against them. Corpus membership itself is pinned
  (`ScenarioCorpus.IsExactlyTheFiveCommittedScenarios`).
- **C5 `world_ref` is loadable.** Worlds load by path or in-memory handle (`world/world_ref.hpp`,
  installed) through the same `validate_world_desc` gate as `WorldBuilder::build()` -- all four
  entry paths (build, file load, path ref, desc ref) validate; the out-of-tree consumer smoke
  (`spade/tests/consumer/`) proves `find_package(spade)` + `load_world_file` on both CI platforms.
- **Substitution is absorbed** (spec Addendum note (iii)). The four migrated `expected_digest`
  values carried over byte-identical from the retired `.digest` files (provenance blocks in each
  scenario file record the chain); the program's only golden regeneration remains Task 3's
  replay_config suffix; `quad_hover` (the fifth scenario) seeded its new digest under gcc-13
  cross-check before finalizing.

World-file schema v1 was frozen at a user checkpoint (strict unknown-keys, `schema_mismatch`
version gate, uniform op-node shape -- all user-ratified 2026-08-12); changes from here are
format-version bumps.

## S6 status (GPU backend, in progress)

S6 (design spec §9/§13) ports the fixed eight-pass substep schedule to a Vulkan/Slang GPU backend
alongside the CPU reference twin, with CPU<->GPU parity as the acceptance bar rather than
GPU-only correctness. As of this note:

- **All eight schedule passes have GPU kernels.** MediumUpdate, ForceElements (rotors + drag),
  CollisionStatic, CollisionDynamic (the sorted-grid broad phase's build/sort/resolve chain), Integrate
  and SensorSynthesis are ported (`engine/shaders/`, `compute/`, see the module map above); Gravity
  and Publish are inert by design on both backends (no dispatch is ever recorded for either). A demo
  scene is no longer refused for an unported pass -- `spade\scripts\demo.ps1 -Scene <scene> -Backend
  vulkan` runs any scene through the ported kernels.
- **CPU<->GPU parity is measured, not assumed**, against the same five-scenario golden corpus
  (`test_gpu_parity.cpp`): `bounce` (a restitution ladder with friction) comes out **BIT-IDENTICAL**
  between backends over 900 steps; every other corpus scenario's worst per-quantity deviation is
  measured on this box's Intel Iris Plus (Vulkan 1.3.215, driver 31.0.101.2125 -- the program's one
  correctness/dev-GPU device) and pinned as a tolerance band with provenance, never widened to make a
  failure pass without a root-caused reason. The bit-portability discipline that makes CPU digests
  reproducible (`fp32_math`, `-ffp-contract=off`) has a GPU analogue: no `OpFDiv`/`sqrt` and no
  GLSL.std.450 transcendental import on any parity-profile Slang module (they route through the
  `log32`/`exp32`/`sin32`/`cos32`/`div32` Slang port instead), `NoContraction` + denormal-preserve
  pinned on every compiled variant, machine-checked by `test_slang_layouts.cpp`'s scanner (rules
  P1-P5, E1/E2).
- **The `workgroup_size` backend knob is live**, not merely plumbed: every kernel compiles once per
  entry in `compute::kSupportedWorkgroupSizes` (`{32, 64, 128}`), the knob reaches both pipeline
  creation and the dispatch group-count divisor, and the resulting variants are bit-identical by
  SHA-256 to a pre-change build (S6 Task 9b) -- the parameterization is provably nothing but the
  local size. A7's backend-knob-invariance test battery (`test_gpu_invariance.cpp`) sweeps it and
  mutation-kills a deliberately local-size-dependent reduction to prove the sweep discriminates.
  The 18-entry registered-state walk (state/arenas.hpp) is unchanged -- S6 adds no `register_array`
  call; every GPU-side buffer is derived, backend-internal storage.
- **The dev-GPU rate envelope is measured and recorded, not gated**, per this program's
  user-ratified posture: `spade/tests/bench/baselines.json`'s `_meta` block carries the full
  account, including the honest dual-basis (`cpu_time` vs. `real_time`) reading for
  `BM_StepQuadWorldsGpu/64` (the literal 64-worlds x 1kHz target scene) -- a bare ~1.014x pass on
  the real-time basis this box's integrated GPU measures. A miss on this specific hardware is a
  fact about this box, not a build-breaking gate (design spec §12's own "misses trigger design
  review, not silent acceptance" -- recorded here, not silently accepted).
- **What is not yet S6**: camera sensors, rendering and `FramePool` (Addendum A8, explicitly S7);
  full heterogeneous per-world geometry sets beyond the two-world regression case S6 Task 6b added
  (`HeterogeneousGeometrySetMatchesSoloRuns`) stay S7+/a future training-spec generator session
  (TR5). The plan's own ledger (`.superpowers/sdd/2026-08-13-spade-engine-s6/progress.md`) is the
  authoritative, task-by-task record while the program is open; this section is a snapshot, not a
  substitute for it.

## v1 (frozen)

`src/`, `include/Spade/`, `examples/` (the `Sandbox` consumer -- see the Demo table above), `assets/`
are **frozen**: nothing here changes except Task 3's dead-code deletions (the root `CMakeLists.txt`'s
`SPADE_BUILD_V1` option comment records the exception clause and its example --
`EnableBruteForceNewtonianGravity`, below). It builds when `SPADE_BUILD_V1` (default `ON`) is set,
and is the renderer `spade_viewer` (above) drives; the `v1-*` demo scenes run it directly and
unmodified.

- **Prerequisites**: C++20, CMake 3.28+ (the real floor -- see the root `CMakeLists.txt`'s own
  comment), OpenGL 4.3 core (compute shaders + SSBOs), Windows (`windows.h`/`psapi.h` for process
  stats; platform unlock is a v2-era item).
- **Dependencies**, fetched at configure time via `FetchContent` (`vendor/CMakeLists.txt` --
  nothing vendored in-tree):

  | Dependency | Pin | Role |
  |---|---|---|
  | GLFW | tag `3.4` | Windowing / input / GL context |
  | GLM | tag `1.0.1` | Mathematics (shared with v2) |
  | GLAD | `libigl/libigl-glad` @ `651a425` | OpenGL loading (4 compute/image entry points hand-loaded on top) |
  | Dear ImGui | tag `v1.91.5-docking` | Linked but not yet used in the engine |

- **API**: `examples/sandbox/main.cpp` is the canonical, always-accurate example -- read it
  directly rather than a transcription here, which is exactly what went stale last time (an
  `EnableBruteForceNewtonianGravity` stub this section used to document was deleted by Task 3
  without the README noticing). For the class/struct surface itself, `include/Spade/Core/Engine.hpp`,
  `Components.hpp` and `Primitives.hpp` are the source of truth; they are short, commented headers,
  not a hand-maintained duplicate.
