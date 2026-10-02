# Interface — status

**The only place that says what exists today.** Checked against the tree at `master` `df33f09` on 2026-10-02 by reading the code, CMake and tests. The measured run is the build of `interface/drone-box-v2` at `67cfb51` (rebased on `ab2a21e`, merged as `df33f09`) in the Interface worktree's `build-ninja/release`: full suite **921 total, 919 passed, 2 skipped, 0 failed**. Per-file test counts are `TEST` macros in the source. Unmerged branches are named where they change a row.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| Installable library (`SL2`) | **Built.** Nine targets are installed and exported as `spade::` (`core`, `world`, `state`, `objects`, `render`, `physics`, `sim`, `vehicles`, `compute`), each with its header directory; `sensors/` headers install alongside `sim`. `render_gl` is built but not installed. `SL2`'s text says "eleven modules"; the count was never reconciled with the tree | `engine/CMakeLists.txt`, `cmake/spadeConfig.cmake.in` |
| Out-of-tree consumer (`SL2`) | **Built, not run.** Nothing has configured it since the 2026-09-28 split (`../backlog.md`) | `tests/consumer/` |
| Release identity (`SL2a`) | **Partly built.** `CHANGELOG.md` and a standalone `README.md` exist; version `0.2.0`; no versioning policy is written down | `CMakeLists.txt` `project(Spade VERSION 0.2.0)` |
| Public-surface rule (`SL2b`) | **Kept by review.** The sandbox links only `spade::` targets (now including `spade::sim`) plus `glfw`, `imgui` and the system `opengl32`/`psapi`, and includes no engine path by relative reach. The guard is **not built** | `sandbox/CMakeLists.txt` |
| Same render path (`SL10`) | **Built.** CPU frames come from `spade::render::render`; the GPU path is the engine's `spade::render_gl`. The headless sink is asserted byte-identical to the render output, with a one-byte-wrong sink as its mutation control | `test_sandbox_target_sink.cpp` (4) |
| The sink seam (`SL11`) | **Built.** `TargetSink`, `HeadlessTargetSink` (PPM), `GlTargetSink` (GLFW 3.4, ImGui `v1.91.5-docking` on OpenGL3, pimpl, refuses with the cause when there is no backend). GPU first, CPU fallback, through shared `start_gpu`/`open_window` helpers used by both scenes | `sandbox/target_sink.hpp`, `gl_target_sink.*`, `main.cpp`; `test_sandbox_camera_and_channels.cpp` (13, incl. the drone orbit) |
| Sandbox build gate (`SL15a`) | **Built as amended.** `SPADE_BUILD_SANDBOX` defaults ON; `SPADE_GLFW_HAS_BACKEND` probes the windowing backend | `CMakeLists.txt`, `vendor/CMakeLists.txt` |
| The builder (old plan C2 and after; `--scene builder`) | **Built.** Place, pick, drag, delete, duplicate; transform and colour edits; orbit camera; HUD with the timing split and memory. Render data only: no bodies, nothing steps, and its "physics running" toggle says so. `--smoke` scripts it through the real window and asserts. Spheres and cylinders face outward and spheres are closed at the poles (`c03bb94`), pinned by a winding test shown to fail under both old defects | `sandbox/builder_scene.hpp`, `main.cpp`; `test_sandbox_builder.cpp` (27) |
| The drone sim box (`INT-1`), the default scene | **Built and merged** (`0752b62` stand and controller; `df33f09` view and wiring). A quadrotor on a CPU test stand (position and velocity pinned bitwise), flown in attitude through the engine's rotor model by a PD controller and a yaw-last mixer; a fixed-dt accumulator; debounced rebuild carrying state; Vulkan refused visibly. The air field (medium + actuator-disc wakes) drawn as a body-aligned viridis slice that keeps a plume pair in view from any azimuth; real-size drone parts. Keys, a 0.5–6 m orbit, the physics panel and readouts; `--scene`/`--view` flags. **Verified headless:** the heatmap shows the downwash columns (0–9.40 m/s, i.e. 2·v_h) and the standard frame is lit from above (disc-top pixels match the predicted top-lit colour to the byte). **Not yet done:** the interactive window check (lead's Task 7). The front rotor's disc in the nose colour is on `interface/front-rotor` (`05651a8`) | `sandbox/drone_sim.hpp`, `drone_view.hpp`; `test_sandbox_drone.cpp` (16), `test_sandbox_drone_view.cpp` (14) |
| Object-centric editor (`SL12`) and scene sources (`SL13`) | **Not built.** Paused by the restructure until the editor spec (old plan tasks C3–C8) | — |
| v1 quarantine (`SL14a`, `engine D10`) | **Not executed, and blocked**: the register's SPH row is open. v1 is present and untouched; `SPADE_BUILD_V1` defaults ON; there is no `legacy/` | `CMakeLists.txt`; `../../v1-transfer-register.md` |
| Successor scenes and baselines (`SL14b`, `SL14c`) | **Not started.** No baselines have been captured from the live v1 tools | — |
| Quadrotor template (`engine D4`, template half) | **Not built.** The quadrotor is engine code (`vehicles/quadrotor.*`, Physics); the drone box's controller and mixer are written as template material | — |
| Demos | `scripts/demo.ps1 -Scene sandbox` (the drone sim box) and `-Scene sandbox-builder` launch `spade_sandbox`, beside the `spade_viewer` and v1 `Sandbox` scenes | `scripts/demo.ps1` |

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
| **The sandbox does not build with `SPADE_RENDER_GL=OFF`** | `sandbox/CMakeLists.txt` defines `SPADE_SANDBOX_HAS_GPU` for exactly that case, but no source reads it: `main.cpp` includes `render_gl/gl_renderer.hpp` and uses `GlRenderer` unconditionally. Guard the include and the GPU calls so the CPU-only build the option promises exists |
| **Vulkan in the drone sim box is refused** | By design until the engine has a translation lock on both backends (`../backlog.md`); behaviors are CPU-only today. Lift the refusal when it lands |
| **The GPU path ignores `RenderOptions`** | Draw modes, overlays and shadows are CPU-only (`render_gl/gl_renderer.cpp`). Rendering's to close; the sandbox shows whichever path is active and does not hide the gap |
| **The superseded record said plan C was unexecuted** | `superseded/2026-09-consolidation/06-sandbox-and-v1.md` and its `07-status.md` describe plan C as written and unexecuted and say no `spade_sandbox` target exists; C0–C2 and the builder had landed. Those files are frozen; this table is the correction |
