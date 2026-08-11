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
one accounted-for, filterable, timed case. 349 tests, both presets, green as of this task.

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

## Module map

`engine/` (v2, C++23, fp32-only state & math, no exceptions across module boundaries,
`std::expected`-based `Result<T>`) -- one CMake target per subdirectory, dependency arrows point
strictly downward (`sim` -> `vehicles`/`physics` -> `world` -> `state` -> `core`; nothing below a
layer knows the layer above it exists):

| Directory | CMake target | What it is |
|---|---|---|
| `core/` | `spade::core` | Typed handles (`ids.hpp`), `Error`/`Result<T>` (`error.hpp`), the `Tick` step counter (`time.hpp`), math primitives (`math_ops.*` -- the exp-map orientation update), the seeded `rng::Stream` (`rng.hpp`, errata-R4 domain-tag discipline). No wall-clock reads anywhere in this tree except `tools/` and bench timers. |
| `ecs/` | *(header-only, no target)* | Generic handle + component-pool + registry primitives (`handle.hpp`/`pool.hpp`/`registry.hpp`), Task 1's ECS scaffold per the design spec's ECS section. Its own test suite (`test_ecs.cpp`), but **not** what the physics step actually runs on -- that is `state/`'s purpose-built arena system below. Kept as a general-purpose component-storage kit. |
| `state/` | `spade::state` | The state backbone: `layout.hpp`'s shared POD structs with `static_assert`ed std430 offsets (the Slang layout single-source, until S6), the world-partitioned `ArenaSet` + `StateRegistry` (`arenas.*`, `registry.*`), and the versioned snapshot/restore blob + file IO (`snapshot.*`) -- the engine's replay/determinism contract. |
| `world/` | `spade::world` | Analytic SDF scene programs (`sdf.*`), the `WorldBuilder` fluent API + its `WorldDesc` product (`builder.*`), and the Dryden turbulence filter (`medium.*`). Worlds are authored in code today; a YAML world file is an S5 item, not yet built. |
| `physics/` | `spade::physics` | The per-substep dynamics passes: `Integrate` (symplectic Euler + specific-force capture, `integrator.*`), `CollisionStatic` (sphere-proxy vs SDF, `contacts.*`), `CollisionDynamic` (world-batched sorted-grid broad phase, `grid.*`), and the `ForceElements` pass's drag law (`forces.*`). Each pass's op order is the CPU/GPU parity contract (D1/D11) -- compiled with `-ffp-contract=off` on non-MSVC so the optimizer cannot silently fuse it. |
| `sim/` | `spade::sim` | The fixed eight-pass substep **schedule** as data (`physics/schedule.*` -- compiles here, not into `spade_physics`, because only `Simulation` runs it), and `Simulation`/`WorldSet` (`sim/*`): arenas, tick, the substep loop, the structural queue, spawn/despawn, snapshot/restore, model registration. This is the engine's one public entry point. |
| `sensors/` | *(compiles into `spade::sim`)* | IMU synthesis (`imu.*`: mount-frame specific force + angular velocity, tick-stamped) and the sensor output ring buffers (`rings.hpp` -- editor tech spec TA5's sensor-poll convention, made executable). |
| `vehicles/` | `spade::vehicles` | The model-type layer (design spec section 6): `RotorElement` (thrust/torque curves, RPM lag, momentum-theory inflow, SDF ground effect -- `rotor.*`), the `ModelType` registry (`model_type.*`), and `Quadrotor` (`quadrotor.*`). "Nothing vehicle-specific below here" -- the dependency arrow points down into `physics`/`world`/`state`, never back. |
| `testing/` | *(header-only, test support only)* | `replay.hpp`: the determinism digest (`state_digest()`, an incremental FNV-1a fold over the state registry's walk order) and the scenario replay harness. Included by `spade/tests/` only -- absent from every install/export rule, not something an out-of-tree consumer has any business with. |
| `tools/` | `spade_viewer` (`SPADE_BUILD_V1`-gated) | The strangler viewer: v1's frozen OpenGL renderer fed by v2 `Simulation` state, one demo scene at a time (`viewer/bridge.*`, `viewer/scenes.cpp`, `viewer/main.cpp`). The one place in `engine/` allowed a wall-clock read (frame pacing) and the one v2 code that includes v1 headers. |

`spade/tests/` (`spade_tests`, gtest, ctest label `spade;T0`) mirrors this module list one
`test_*.cpp` per concern, plus `test_m1b_bar.cpp` (the M1B charter bar as executable asserts) and
the golden determinism corpus (`tests/golden/*.digest`, four scenarios: `ballistic`, `bounce`,
`shower`, `two_world_isolation`). `spade/tests/bench/` (`spade_bench`, google-benchmark, **not**
ctest -- see its own header comments) holds `bench_core.cpp` (math primitive throughput) and
`bench_sim.cpp` (`Simulation::step()` throughput sweeps); `baselines.json` records reference runs.

## M1B status

The charter's M1B bar -- fixed-step, headless, seeded determinism, snapshot/restore, 6-DOF +
Quadrotor + IMU -- is met on the **CPU path** (the charter sequences the GPU-path rows,
snapshot-GPU and instance-isolation-GPU, to S6; M1B is CPU by design, not by omission).
`test_m1b_bar.cpp`'s five charter-bullet tests, its summary table (a sixth TEST that re-derives and
prints all five results -- not a seventh charter bullet), and one Addendum A3 conformance case
(`ResetPreservesRosterRoundTrip`, see below) are all part of the 349-test suite above -- seven
TESTs total in that one file -- both presets green.

- **Version**: Spade is at **0.2.0** as of this claim (charter §5's version promise, Addendum
  A11(i)) -- this is Spade-the-project's own release number and is separate from the snapshot
  format's own version (`state/snapshot.hpp`'s `kSnapshotVersion`, currently `1`, a real symbol
  today) and from the world-FILE format's eventual version (an S5 concept -- the YAML world file
  does not exist yet, so it has nothing to version). Neither is touched by this bump.
- **Rate envelope** (design spec §12): both CPU-measurable targets are **HIT** on this box
  (`spade/tests/bench/baselines.json` has the full run) -- a single world's quadrotor+IMU scene
  measures ~296,000 substeps/sec against a >=10,000 floor, and the 64-world batched set measures
  ~176,000 world-substeps/sec against a >=64,000 (64 worlds x 1 kHz) floor. The spec's batched
  target is phrased "on the dev GPU"; this is the CPU reference twin, so it is a strong signal, not
  the literal S6 measurement.
- **Addendum A3 conformance cases**: 1/2 written. `ResetPreservesRosterRoundTrip`
  (`test_m1b_bar.cpp`) covers the reset-preserves-roster case with today's engine surface
  (`restore()` into an earlier snapshot on the same `Simulation` IS a reset). The other case,
  reseed determinism, needs new engine surface (`Simulation::reseed(seed)` does not exist --
  `WorldParams::seed` is fixed at `create()` and never rewritten) and is deliberately **not** built
  during close-out -- ticketed for S5/M1b (`design-specs/kat-open-agendas.md`'s T21 section, local
  and untracked).
- **What is NOT M1B**: the Vulkan/Slang GPU backend and real rendering (S6+), the YAML world file
  (S5 -- worlds are authored in code today via `WorldBuilder`), Linux GPU support (rides the Vulkan
  backend), and the `config`/`dt_ns`/`substeps` gap in `restore()`'s replay contract (a hard S5
  prerequisite, ticketed alongside the reseed case above).

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
