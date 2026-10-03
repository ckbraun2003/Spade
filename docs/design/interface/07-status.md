# Interface — status

**The only place that says what exists today.** Checked against the tree at `master` `332a186` on 2026-10-02 by reading the code, CMake and tests. The last measured Interface run is `interface/front-rotor` (`e5660dd` on `df33f09`, merged as `b7616c1`) in the Interface worktree's `build-ninja/release`: full suite **921 total, 919 passed, 2 skipped, 0 failed**. Per-file test counts are `TEST` macros in the source. No Interface branch is open. **Updated at `883c0cc` for the user's 2026-10-02 rulings** (`INT-3`, `INT-4`, `TD-11`). No Interface code has changed since `332a186`.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| Installable library (`SL2`) | **Built.** Nine targets are installed and exported as `spade::` (`core`, `world`, `state`, `objects`, `render`, `physics`, `sim`, `vehicles`, `compute`), each with its header directory; `sensors/` headers install alongside `sim`. `render_gl` is built but not installed. `SL2`'s text says "eleven modules"; the count was never reconciled with the tree | `engine/CMakeLists.txt`, `cmake/spadeConfig.cmake.in` |
| Out-of-tree consumer (`SL2`) | **Built; run once by hand, and by no gate.** Core ran it against a `SPADE_VULKAN=OFF` install while fixing restructure defect 3. It failed to compile at `b7616c1` and built and ran at `6d40740` (merged as `3a48c4d`). The Docker leg (`TD-11`) is to run it with Vulkan ON and OFF | `tests/consumer/` |
| Release identity (`SL2a`) | **Partly built.** `CHANGELOG.md` and a standalone `README.md` exist; version `0.2.0`; no versioning policy is written down | `CMakeLists.txt` `project(Spade VERSION 0.2.0)` |
| Public-surface rule (`SL2b`) | **Kept by review.** The sandbox links only `spade::` targets (now including `spade::sim`) plus `glfw`, `imgui` and the system `opengl32`/`psapi`, and includes no engine path by relative reach. The guard is **not built** | `sandbox/CMakeLists.txt` |
| Same render path (`SL10`) | **Built.** CPU frames come from `spade::render::render`; the GPU path is the engine's `spade::render_gl`. The headless sink is asserted byte-identical to the render output, with a one-byte-wrong sink as its mutation control | `test_sandbox_target_sink.cpp` (4) |
| The sink seam (`SL11`) | **Built.** `TargetSink`, `HeadlessTargetSink` (PPM), `GlTargetSink` (GLFW 3.4, ImGui `v1.91.5-docking` on OpenGL3, pimpl, refuses with the cause when there is no backend). GPU first, CPU fallback, through shared `start_gpu`/`open_window` helpers used by both scenes | `sandbox/target_sink.hpp`, `gl_target_sink.*`, `main.cpp`; `test_sandbox_camera_and_channels.cpp` (13, incl. the drone orbit) |
| Sandbox build gate (`SL15a`) | **Built as amended.** `SPADE_BUILD_SANDBOX` defaults ON; `SPADE_GLFW_HAS_BACKEND` probes the windowing backend | `CMakeLists.txt`, `vendor/CMakeLists.txt` |
| The builder (old plan C2 and after; `--scene builder`) | **Built.** Place, pick, drag, delete, duplicate; transform and colour edits; orbit camera; HUD with the timing split and memory. Render data only: no bodies, nothing steps, and its "physics running" toggle says so. `--smoke` scripts it through the real window and asserts. Spheres and cylinders face outward and spheres are closed at the poles (`c03bb94`), pinned by a winding test shown to fail under both old defects | `sandbox/builder_scene.hpp`, `main.cpp`; `test_sandbox_builder.cpp` (27) |
| The drone sim box (`INT-1`), the default scene | **Built and merged** (`0752b62` stand and controller; `df33f09` view and wiring). A quadrotor on a CPU test stand (position and velocity pinned bitwise), flown in attitude through the engine's rotor model by a PD controller and a yaw-last mixer; a fixed-dt accumulator; debounced rebuild carrying state; Vulkan refused visibly. The air field (medium + actuator-disc wakes) drawn as a body-aligned viridis slice that keeps a plume pair in view from any azimuth; real-size drone parts. Keys, a 0.5–6 m orbit, the physics panel and readouts; `--scene`/`--view` flags. The front rotor's disc wears the nose colour so the heading reads from above (`b7616c1`). **Verified headless:** the heatmap shows the downwash columns (0–9.40 m/s, i.e. 2·v_h) and the standard frame is lit from above (disc-top pixels match the predicted top-lit colours to the byte). **Verified in the window** by the lead (plan Task 7, GPU path): the default scene, the complete panel, V, the arrows and D, a clean exit, and the HUD reading the framebuffer size | `sandbox/drone_sim.hpp`, `drone_view.hpp`; `test_sandbox_drone.cpp` (16), `test_sandbox_drone_view.cpp` (14) |
| Object-centric editor (`SL12`) and scene sources (`SL13`) | **Not built.** Paused by the restructure until the editor spec (old plan tasks C3–C8) | — |
| v1 quarantine (`SL14a`, `engine D10`) | **Not executed, and blocked**: the register's SPH row is open. v1 is present and untouched; `SPADE_BUILD_V1` defaults ON; there is no `legacy/` | `CMakeLists.txt`; `../../v1-transfer-register.md` |
| Successor scenes and baselines (`SL14b`, `SL14c`) | **Not started.** No baselines have been captured from the live v1 tools. `INT-4` rules that they be captured now, while v1 builds | — |
| Quadrotor template (`engine D4`, template half) | **Not built.** The quadrotor is engine code (`vehicles/quadrotor.*`, Physics); the drone box's controller and mixer are written as template material | — |
| Demos | `scripts/demo.ps1 -Scene sandbox` (the drone sim box) and `-Scene sandbox-builder` launch `spade_sandbox`, beside the `spade_viewer` and v1 `Sandbox` scenes | `scripts/demo.ps1` |

## Next for Interface

In the backlog's order (`../backlog.md`, "Suggested order"):

1. **The consumer half of the Docker leg** (`TD-11`, Test/Docs's leg): build and run `tests/consumer/` against an installed prefix, `SPADE_VULKAN` ON and OFF.
2. **The `SL14b` v1 baselines** (`INT-4`).
   - **Spec and plan first**, in `plans/`.
   - **Scope** (`03-v1-retirement.md`): the eight `spade_viewer` scenes get state trajectories, reference frames, and `spade_bench` step time and memory. The three v1 `Sandbox` scenes get frames and fresh baselines.
   - **It cannot wait:** the capture is only possible while v1 builds.
3. **Lift the drone box's Vulkan refusal** once Core's translation lock lands on both backends.
4. **The editor spec** (`INT-3`), after Core's module API.
   - The sandbox grows into Spade's editor (`01-editor.md`, "The target"). Old plan tasks C3–C8 are the starting inventory, rethought in engine-model terms.
   - Saved scenes are world files, and the schema they need is Core's (`01-editor.md`).

**At any time:** the `SPADE_RENDER_GL=OFF` build (debt below).

## Open items — needs a user decision

**None.** The user ruled both on 2026-10-02:
- `INT-3`: the editor's first cut saves and loads world files only.
- `INT-4`: capture the `SL14b` baselines now.

## Open items — debt

| Item | Detail |
|---|---|
| **`SL2b` has no guard** | The rule is kept by review. The guard (a seeded reach into an engine internal must fail it) is `SL18` obligation 6, Test/Docs's harness and this realm's subject |
| **`SL2`'s module count** | The ruling says eleven modules; the tree installs nine targets. Correct the count, or name the missing two |
| **No versioning policy** (`SL2a`) | `0.2.0` has a changelog but no stated rule for when the version moves |
| **No gate runs the consumer** | Shared with Test/Docs (`TD-11`, `../backlog.md`). Restructure defect 3 (a `SPADE_VULKAN=OFF` install shipping a header that includes uninstalled `compute/` headers) is fixed on master (`3a48c4d`). The fix was shown only by Core's hand run; the Docker leg is what keeps it fixed |
| **The sandbox does not build with `SPADE_RENDER_GL=OFF`** | `sandbox/CMakeLists.txt` defines `SPADE_SANDBOX_HAS_GPU` for exactly that case, but no source reads it: `main.cpp` includes `render_gl/gl_renderer.hpp` and uses `GlRenderer` unconditionally. Guard the include and the GPU calls so the CPU-only build the option promises exists |
| **A GL-OFF window still asks for GL 4.3** | `gl_target_sink.cpp` requests a 4.3 core context because `GlRenderer` needs SSBOs. A `SPADE_RENDER_GL=OFF` sandbox (`interface/render-gl-off`, in review) has no `GlRenderer`, and its window needs only what ImGui's GL3 backend needs. On a GL 3.3-only machine that window still refuses. Follow-up: request 4.3 only when `SPADE_SANDBOX_HAS_GPU` |
| **Vulkan in the drone sim box is refused** | By design until the engine has a translation lock on both backends (`../backlog.md`); behaviors are CPU-only today. Lift the refusal when it lands |
| **The GPU path ignores `RenderOptions`** | Draw modes, overlays and shadows are CPU-only (`render_gl/gl_renderer.cpp`). Rendering's to close; the sandbox shows whichever path is active and does not hide the gap |
| **The superseded record said plan C was unexecuted** | `superseded/2026-09-consolidation/06-sandbox-and-v1.md` and its `07-status.md` describe plan C as written and unexecuted and say no `spade_sandbox` target exists; C0–C2 and the builder had landed. Those files are frozen; this table is the correction |
