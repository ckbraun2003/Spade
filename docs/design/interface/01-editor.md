# The editor

`spade_sandbox` is Spade's reference application and grows into its editor: the place a physics or rendering change becomes visible rather than inferred from an assertion, and where a developer composes and inspects a world. This document says what every editor feature must keep, and what the sandbox has today. What exists is in `07-status.md`.

## The rules every editor feature keeps

**1. Headless first; the window is a dumb shell.** (`SL15b`, Test/Docs's, quoted.) Every panel is a view over an operation that already exists and is tested without a display. The logic lives in display-free headers (`sandbox/*.hpp`) that `spade_tests` includes directly; the window turns input into calls and draws the panel. Logic that migrates into the window is a defect, because it becomes untestable by construction. A task that cannot state the headless command or test that exercises its logic is not ready to start.

**2. One render path.** (`SL10`.) The sandbox renders through the same `spade::render` entry points the tests use, and the GPU path through the engine's own `spade::render_gl`. There is no sandbox-only shading. A tool that can disagree with the tests is worse than no tool, because it will be believed. The CPU path is the reference; when the GPU path falls short of it, that is Rendering's gap, shown rather than papered over.

**3. The window presents a buffer; it does not render.** (`SL11`.) The seam is `TargetSink::accept(const RenderTarget&)` (`sandbox/target_sink.hpp`): `HeadlessTargetSink` writes PPM frames (the CI path) and `GlTargetSink` uploads the frame to a GLFW window with ImGui on its OpenGL3 backend. The sink never owns frame memory and never decides when the loop ends; closing the window is the application's business. The GPU path draws straight into the window and presents through the same overlay, so both paths show the same panels and the same timing split.

**4. GPU first, CPU fallback, and a missing capability refuses.** The GPU renderer is used when it can be created; otherwise the sandbox says why and runs on the CPU rasterizer. A build with no windowing backend refuses to open a window and names the cause; `--headless` still works. Nothing degrades silently.

**5. Only the public API.** (`SL2b`, see `02-library-surface.md`.) If the editor needs something the installed headers do not offer, that is a finding for Core.

**6. Configuration changes rebuild, debounced, and carry state.** (`INT-2`, proposed.) Anything in a Simulation's `config_hash` is fixed per Simulation, so changing it means a new one. The editor rebuilds only when the user lets go of a control, carries the state that survives the change, and on a refusal keeps the running simulation, shows the reason and puts the control back.

## The scenes

**The drone sim box** (`INT-1`, the default once it lands; it is being built, see `07-status.md`). A quadrotor on a test stand in an empty world, flown in attitude, built to show end to end that what Spade claims works: quadrotor, rotor model, integrator, Dryden turbulence, CPU stepping and both render paths.
- *Stand:* two CPU behaviors hold the body at the origin (kinematic: position and velocity zero; force: `force_acc = −m·g`, so Integrate's `+g` cancels exactly for a 1 kg airframe). The rotor model computes thrust, torque and inflow as for a free vehicle; only the translation is discarded. The IMU reads +g, as a stand-mounted sensor does.
- *Control:* a PD attitude controller and a plus-layout mixer, with yaw limited last so tilt keeps its authority, turn key-set targets into four shaft speeds. Scene code and template material, not engine API.
- *Stepping:* a fixed-dt accumulator (2 ms, 2 substeps, at most 100 steps a frame, backlog dropped) over the window's frame time. Nothing a test steps reads a wall clock.
- *Vulkan:* refused with a message, because behaviors are CPU-only today and the stand would step unpinned. It lifts when the engine has a translation lock on both backends (`../backlog.md`).
- *View:* the drone as real-size parts (lit parts are never non-uniformly scaled), and a camera-facing heatmap slice of air speed: the engine's medium sample plus an analytic actuator-disc wake per rotor (`PHY-3`), binned into 32 unlit viridis materials. The field is read only for drawing.
- *Code:* `sandbox/drone_sim.hpp`, `sandbox/drone_view.hpp`; tests `test_sandbox_drone.cpp`, `test_sandbox_drone_view.cpp`. Design and plan: `../plans/2026-10-01-drone-sim-box-*.md`.

**The builder** (`--scene builder`). Place, pick, drag, delete and duplicate boxes, spheres and cylinders on a ground plane; edit transform and colour. Render data only: builder objects are not bodies and nothing steps. `--smoke` drives it from a script through the real window and renderer, with assertions, and exits non-zero on any failure. Code: `sandbox/builder_scene.hpp`; tests `test_sandbox_builder.cpp`.

## The target: an editor in engine-model terms

The editor is the object-centric tool `SL12` describes, rebuilt in the engine model's terms (`../01-engine-model.md`): it composes worlds from regions, objects and modules; it places templates; it switches a module's fidelity tier or backend on a running world; and it inspects state, fields and channels. `SL13`'s scene sources (presets, test scenarios, saved editor scenes, never `.world.yaml` or consumer recipes) carry over unchanged.

It waits for its own spec (restructure §5; `../backlog.md`, "Sandbox → editor"). The old plan's task list (C3 scene picker, C4 hierarchy and inspector, C5 add object and component, C6 live controls, C7 pause and step, C8 the purity guard) is the starting inventory, not the plan; its deferrals (saved scenes, asset browser, CPU/GPU lockstep view, debug draw, SDF views, scrub) are still the honest list of what would make it feel like an editor rather than a debugger. See `plans/README.md`.

Two things the spec must decide rather than discover:
- **Structural edits on a running world.** Object-graph changes queue to step boundaries once objects take part in stepping (engine model, inherited obligations). Today the graph mutates immediately and nothing holds one while stepping.
- **A reserved type in a menu.** `ComponentTypeId::fluid` is declared for SPH but cannot be constructed. Availability belongs beside the type table (Core), so every enumerator — a menu, a loader, a CLI listing — shows it as reserved with its reason instead of offering it.
