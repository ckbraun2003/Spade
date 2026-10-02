# Interface — status

**The only place that says what exists today.** Checked against the tree at `master` `bfc541c` on 2026-10-02 by reading the code, CMake and tests, plus one measured build of branch `interface/drone-box` (`57a84c3`) in its own worktree's `build-ninja/release`. Test counts are `TEST` macros in the source unless a ctest run is named. Unmerged branches are named where they change a row.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| Installable library (`SL2`) | **Built.** Nine targets are installed and exported as `spade::` (`core`, `world`, `state`, `objects`, `render`, `physics`, `sim`, `vehicles`, `compute`), each with its header directory; `sensors/` headers install alongside `sim`. `render_gl` is built but not installed. `SL2`'s text says "eleven modules"; the count was never reconciled with the tree | `engine/CMakeLists.txt`, `cmake/spadeConfig.cmake.in` |
| Out-of-tree consumer (`SL2`) | **Built, not run.** Nothing has configured it since the 2026-09-28 split (`../backlog.md`) | `tests/consumer/` |
| Release identity (`SL2a`) | **Partly built.** `CHANGELOG.md` and a standalone `README.md` exist; version `0.2.0`; no versioning policy is written down | `CMakeLists.txt` `project(Spade VERSION 0.2.0)` |
| Public-surface rule (`SL2b`) | **Kept by review.** The sandbox links only `spade::` targets plus `glfw`, `imgui` and the system `opengl32`/`psapi`, and includes no engine path by relative reach. The guard is **not built** | `sandbox/CMakeLists.txt` |
| Same render path (`SL10`) | **Built.** CPU frames come from `spade::render::render`; the GPU path is the engine's `spade::render_gl`. The headless sink is asserted byte-identical to the render output, with a one-byte-wrong sink as its mutation control | `test_sandbox_target_sink.cpp` (4) |
| The sink seam (`SL11`) | **Built.** `TargetSink`, `HeadlessTargetSink` (PPM), `GlTargetSink` (GLFW 3.4, ImGui `v1.91.5-docking` on OpenGL3, pimpl, refuses with the cause when there is no backend). GPU first, CPU fallback | `sandbox/target_sink.hpp`, `gl_target_sink.*`; `test_sandbox_camera_and_channels.cpp` (10) |
| Sandbox build gate (`SL15a`) | **Built as amended.** `SPADE_BUILD_SANDBOX` defaults ON; `SPADE_GLFW_HAS_BACKEND` probes the windowing backend | `CMakeLists.txt`, `vendor/CMakeLists.txt` |
| The builder (old plan C2 and after) | **Built.** Place, pick, drag, delete, duplicate; transform and colour edits; orbit camera; HUD with the timing split and memory. Render data only: no bodies, nothing steps, and its "physics running" toggle says so. `--smoke` scripts it through the real window and asserts | `sandbox/builder_scene.hpp`, `main.cpp`; `test_sandbox_builder.cpp` (26) |
| The drone sim box (`INT-1`) | **Being built** on `interface/drone-box`. Task 4 (stand, controller, mixer, accumulator, rebuild) is built and measured: `SandboxDrone` 16/16, full suite 913 total, 880 passed, 0 failed, 33 skipped (the 897 baseline plus 16; the 31 extra skips are the KAT agreement cases, which look for `../KAT` and find none beside a worktree). Task 5 (air field, heatmap, drone parts) and Task 6 (input, camera, panel, flags) are written, unbuilt, and wait on `core/drone-medium`, `physics/rotor-wake` and `rendering/sun-convention` | `sandbox/drone_sim.hpp`; `test_sandbox_drone.cpp` (16) |
| Object-centric editor (`SL12`) and scene sources (`SL13`) | **Not built.** Paused by the restructure until the editor spec (old plan tasks C3–C8) | — |
| v1 quarantine (`SL14a`, `engine D10`) | **Not executed, and blocked**: the register's SPH row is open. v1 is present and untouched; `SPADE_BUILD_V1` defaults ON; there is no `legacy/` | `CMakeLists.txt`; `../../v1-transfer-register.md` |
| Successor scenes and baselines (`SL14b`, `SL14c`) | **Not started.** No baselines have been captured from the live v1 tools | — |
| Quadrotor template (`engine D4`, template half) | **Not built.** The quadrotor is engine code (`vehicles/quadrotor.*`, Physics) | — |
| Demos | `scripts/demo.ps1` launches `spade_viewer` and v1's `Sandbox` only; it has **no `spade_sandbox` entry** | `scripts/demo.ps1` |

## Open items — needs a user decision

1. **Sign `INT-1`** (the drone sim box as the default scene; approved in conversation 2026-10-01) and **`INT-2`** (debounced rebuild, refusal keeps the running simulation).
2. **Saved scenes in the editor's first cut.** The old plan deferred them, while its checkpoint named "a saved scene". The editor spec should settle this with the user up front rather than at the checkpoint.
3. **When to capture the `SL14b` baselines from live v1.** They can only be taken while v1 builds, and nothing schedules them.

## Open items — debt

| Item | Detail |
|---|---|
| **`SL2b` has no guard** | The rule is kept by review. The guard (a seeded reach into an engine internal must fail it) is `SL18` obligation 6, Test/Docs's harness and this realm's subject |
| **`SL2`'s module count** | The ruling says eleven modules; the tree installs nine targets. Correct the count, or name the missing two |
| **No versioning policy** (`SL2a`) | `0.2.0` has a changelog but no stated rule for when the version moves |
| **The consumer smoke is not run** | Shared with Test/Docs (`../backlog.md`). It is also how restructure defect 3 (a `SPADE_VULKAN=OFF` install shipping a header that includes uninstalled `compute/` headers) gets verified once Core fixes it |
| **No `spade_sandbox` in `demo.ps1`** | The demo script predates the sandbox |
| **The GPU path ignores `RenderOptions`** | Draw modes, overlays and shadows are CPU-only (`render_gl/gl_renderer.cpp`). Rendering's to close; the sandbox shows whichever path is active and does not hide the gap |
| **KAT-citing comments in Interface code** | Five lines: `sandbox/CMakeLists.txt`, `tests/consumer/CMakeLists.txt`, `tests/consumer/main.cpp` (two). Swept on the Interface branch with the drone-box work |
| **The superseded record said plan C was unexecuted** | `superseded/2026-09-consolidation/06-sandbox-and-v1.md` and its `07-status.md` describe plan C as written and unexecuted and say no `spade_sandbox` target exists; C0–C2 and the builder had landed. Those files are frozen; this table is the correction |
