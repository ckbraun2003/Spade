# Spade

**A general-purpose, deterministic simulation and rendering engine** (C++23). Compose worlds from objects and modules, step them headless at a fixed rate, batch many worlds in one call, snapshot and replay them bit-for-bit, and render them offscreen.

```cpp
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"

// Author a world: a ground plane, room for a few bodies.
spade::Result<spade::WorldDesc> world = spade::WorldBuilder()
    .name("drop")
    .environment(spade::Environment{})
    .capacities(spade::Capacities{4, 4, 1, 1})
    .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
    .build();

spade::WorldInstanceDesc prototype;
prototype.world = *world;

// 64 copies, each with its own seed derived from one scene seed.
spade::WorldSetDesc set = spade::replicate(prototype, /*count=*/64, /*scene_seed=*/0xC0FFEEULL);

// One Simulation steps all 64. dt and substeps are fixed for its lifetime.
spade::Result<spade::Simulation> sim =
    spade::Simulation::create(set, /*dt_ns=*/2'000'000, /*substeps=*/2);

spade::BodySpawn ball;
ball.pos = glm::vec3(0.0f, 5.0f, 0.0f);
ball.mass = 1.0f;
ball.inv_inertia_diag = glm::vec3(10.0f);
spade::Result<spade::BodyRef> ref = sim->spawn(/*world_index=*/0, ball);

sim->step(100);                                         // every world, batched
spade::Result<spade::SnapshotBlob> blob = sim->snapshot();  // restore() refuses a blob
                                                        // taken under another dt/substeps
```

Every fallible call returns `Result<T>` (`std::expected`-based); no exception crosses a module boundary. `tests/consumer/main.cpp` is the same idea as a complete out-of-tree program.

## What it promises

The charter's laws (`docs/design/00-charter.md`), in short:

- **Fixed step and seeded.** No clock in the step path. Same seed, same inputs, same bits.
- **Snapshot, restore and replay are first-class,** and restoring under a different configuration is refused.
- **Every module declares a grade:** reference (a CPU implementation and a golden), banded (a measured band against the reference) or best-effort. The GPU is never a golden source.
- **No silent fallback.** An unavailable backend or module is refused or announced.
- **Batching many worlds is native,** and stepping never depends on rendering.
- **No domain in the core.** Vehicles, sensors and scenes are modules and templates on the public API.

## Build and test

Windows, MSVC, CMake 3.28+, Ninja. The scripts are PowerShell; run them in the foreground.

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build.ps1                 # release
powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Preset debug
powershell -ExecutionPolicy Bypass -File scripts\test.ps1                  # ctest -L spade
```

The first build fetches its dependencies. `scripts\test.ps1` is the gate: green on both presets. The current measured counts, with the tree and commit they came from, are in `docs/design/test-docs/07-status.md`.

## Run

`build-ninja\release\bin\spade_sandbox.exe` (the **Spade Builder** desktop shortcut on the development machine) opens the sandbox on its default scene, the drone sim box: a quadrotor held in place while the air moves around it, with an air-velocity view (`V`) and a physics panel. `--scene builder` opens the object builder instead, and `--headless` renders one frame with no window. `--help` lists the rest.

`scripts\demo.ps1 -Scene <name>` builds if needed and launches a scene: `sandbox` or `sandbox-builder` in the sandbox, or one of the viewer scenes `drop`, `bounce`, `shower`, `gate`, `hover`, `wind`, `flight`, `swarm`.

## Layout

| Directory | What it is |
|---|---|
| `engine/` | The engine: `core/ state/ world/ physics/ objects/ sim/ sensors/ vehicles/ render/ render_gl/ compute/ shaders/`, mostly one CMake target per directory with dependencies pointing strictly downward; `testing/` is test support and `tools/` the viewer |
| `tests/` | The test suite, the golden corpus, the bench, and `tests/consumer/` (an out-of-tree `find_package(spade)` example) |
| `sandbox/` | The sandbox application, which grows into Spade's editor |
| `scripts/` | Build, test and demo scripts, and the pre-push guard |
| `docs/design/` | The design library: charter, engine model, and one library per realm |
| `src/ include/ examples/ assets/` | The frozen v1 OpenGL engine, kept until its last system (SPH fluid) is transferred (`docs/v1-transfer-register.md`) |

## Where to read next

| | |
|---|---|
| `AGENTS.md` | The short guide for anyone, human or agent, changing this code |
| `CONTRIBUTING.md` | The rules that protect determinism and parity |
| `docs/design/INDEX.md` | "To do X, read Y" across the design library |
| `docs/design/<realm>/07-status.md` | What exists today, realm by realm |
| `CHANGELOG.md` | What changed |

Spade began inside the KAT monorepo and became its own repository on 2026-09-28. KAT is now a consumer; see `docs/design/consumers.md`.
