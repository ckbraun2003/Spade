# Retiring v1

The frozen v1 OpenGL engine (`src/`, `include/`, `examples/`, `assets/`) and `spade_viewer` (`engine/tools/viewer/`, which renders v2 physics through a v1 bridge) are still in the tree. This document says what has to be true before they leave, and how they leave. What exists is in `07-status.md`.

## The rule (`SL7`, `engine D10`)

v1 may not be quarantined while it is the only implementation of anything. Every v1 system carries exactly one of four dispositions, and a silent drop is not one of them:

| Disposition | Meaning | Blocks quarantine |
|---|---|---|
| `transferred` | A v2 implementation exists and is tested | no |
| `to-transfer` | v1 is still the only implementation | **yes** |
| `retired-with-reason` | Deliberately not transferred; the reason is recorded | no |
| `retired-to-sandbox` | Leaves the engine by design; the capability lives in the sandbox | no |

The register is `../../v1-transfer-register.md`, and it is the authority, not a copy. `test_transfer_register.cpp` parses it (the guard is Test/Docs's half of `SL7`), so the gate is evaluated by a test rather than by a reader. Until the register has no `to-transfer` row, v1 stays in the tree untouched, built behind `SPADE_BUILD_V1`, and nothing new may depend on it (`engine D10`).

**The one open row is SPH fluid** (`SL8`, Physics). It returns as a field provider once Core's module API exists (restructure §5). Everything below waits on it.

## The per-system dispositions (`SL9a`–`SL9f`)

The rows are in the register; the reasons are in `00-decisions.md`. In short: the sorted grid superseded brute-force collision; custom shaders have no meaning on a CPU rasterizer and wait for a GPU render backend; velocity render mode, the camera component and the instancing helpers transferred; colour did not, because it is a material, not a body property; barycentric wireframe is unnecessary when the rasterizer draws wireframe directly. **`InputComponent` is the one `retired-to-sandbox` row**: input is an application concern, the engine has no input concept, and the capability survives as the sandbox's camera controller and key handling.

## How v1 leaves (`SL14a`)

**Quarantine, not deletion.** The v1 paths, `engine/tools/viewer/` and the `SPADE_BUILD_V1` option move to `legacy/` as pure renames with zero edits. GLFW and ImGui move from the v1 gate to the sandbox gate; GLAD leaves the build. "Cut off" is enforced: no build wiring references `legacy/`, nothing links it, no test includes it, and a ratchet guard fails on a seeded reach back in. The render spec's older "deleted" wording (`RS10b`) is dead text.

**Harvest first.** `engine/tools/viewer/scenes.cpp` is v2 code, not v1; only `bridge.cpp` touches the old engine. Its eight scene constructors are ported forward as successor presets before the directory moves. So is `setup.cpp`, the viewer's simulation setup, which `INT-4` moved out of `bridge.cpp` unchanged. Both build as the v1-free `spade_viewer_scenes` library, which the trajectory guard also links. They are harvested forward, not quarantined.

## Successor scenes, and the step that cannot be repeated (`SL14b`, `SL14c`)

Each of the eleven retired scenes gets a successor preset that exercises the same objects, physics and rendering. They are not numerically exact reproductions, and they are judged on three axes that differ by population:

| Axis | The eight `spade_viewer` scenes (v2 physics) | The three `Sandbox` scenes (v1 engine) |
|---|---|---|
| Functional | Bit-identical state trajectories across the port | The successor exercises the same systems and runs stably; no trajectory exists to compare |
| Visual | Reference frames from the live tools, judged on geometry, motion and materials (never a pixel diff); new frames are then pinned as goldens | Same |
| Latency and memory | Physics step time and memory via `spade_bench` | Fresh baselines only |

Render time is not comparable on either population; the CPU path is slower than hardware GL by design.

**The ordering is the binding constraint.** Baselines can only be captured from the live v1 tools, so the sequence is: characterise each scene, capture baselines, build the successors, compare, and only then quarantine. Once v1 is cut from the build the left-hand side of every comparison is gone for good.

**The baselines are captured** (`INT-4`, 2026-10-03, merged `0935c6f`; plan `plans/2026-10-02-v1-baselines.md`). The characterisation and capture steps are done:
- **Functional:** the eight viewer scenes' trajectories are CPU goldens in `tests/golden/viewer/` (`TD-1`). They come from the viewer's own setup, stepped headless (`spade_viewer <scene> cpu --trajectory`).
- **Visual:** reference frames of all eleven scenes, at wall-clock moments, are in the archive `tests/v1-baselines/`. The archive is never asserted and never regenerated.
- **Performance:** physics step time and memory for the viewer scenes, and the `Sandbox`'s real frame rate and memory, are in the same archive.

**One deviation from the axes table:** the viewer scenes' step time and memory come from the viewer's headless mode, not `spade_bench`. The bench cannot build `scenes.cpp` without reaching into a v1-gated tool, and it measures no memory. The headless mode uses the same constructors, setup and step.

The 50,000-body fluid scene stays as a permanent preset and a stated capability target, labelled a capability demonstration, never a parity or regression claim (`SL14c`).
