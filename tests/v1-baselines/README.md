# v1 baselines archive (`INT-4`, `SL14b`)

**This directory is a record. Nothing asserts it, and nothing regenerates it.** The frames come from the live v1 renderer, which cannot run after quarantine. If something here turns out wrong, annotate this README; do not re-capture.

The eight viewer scenes' trajectories are not here. They are CPU goldens in `tests/golden/viewer/`, asserted by the trajectory guard and governed by `TD-1` (Test/Docs's ruling, 2026-10-02).

Successor scenes are compared against both (`SL14b`, P6). Spec and plan: `docs/design/interface/plans/2026-10-02-v1-baselines.md`.

## Provenance

| | |
|---|---|
| Captured | 2026-10-03, 00:22–00:26 (UTC−4), in 206 s |
| Commit | `511bc39` on `interface/v1-baselines`. `spade_viewer.exe` was built at that commit in the same slot |
| `spade_viewer.exe` | SHA-256 `630832ec…2797` (full hash in each viewer `capture.json`) |
| `Sandbox.exe` | SHA-256 `21a28909…468b`, built 2026-10-01 from frozen v1 |
| Machine | DESKTOP-N7QJD3C, Windows 11 Home |
| GPU and driver | Intel Iris Plus Graphics, driver 31.0.101.2125 |
| Toolchain | MSVC 14.44.35207 (`cl` 19.44), Release |
| Command | `powershell -ExecutionPolicy Bypass -File engine\tools\viewer\capture-v1-baselines.ps1` (all eleven scenes, both parts, default directories) |
| Load | No build or test ran during the capture. The lead held a slot for it |

## What happened during the capture

- **43 of 44 frames came through `PrintWindow`.** `sandbox/fluid/paused.png` came back blank from `PrintWindow`. The blank check caught it, and the harness retook it from the screen. Each `capture.json` names the method per frame.
- **Play started on the first motion seen:** cubes after a 60 ms hold, spheres after 120 ms, and fluid after 250 ms. Holds that showed no motion are logged in `play_attempts` and were never labelled "playing". Paused motion was 0 for all three scenes.
- **Real `Sandbox` frame rates during play:** fluid 1.34, spheres 2.91 and cubes 3.67 frames/s. v1 printed 78.2, 22.7 and 24.2 FPS for the same runs (see "How to read the performance numbers").
- **shower's physics runs slower than real time** on this machine: a median of 17.6 ms per 4 ms tick in this run. Its viewer frames therefore show less simulated time than their wall-clock label. The step time also varies between runs: 11.7 ms in an earlier run the same night.
- **The trajectories were checked before the capture:** two runs of each scene matched byte for byte, and each golden's body equals that run.

## What each file is

| File | What it records | Deterministic |
|---|---|---|
| `performance.json` | Physics step time (median and p90 after 100 warm-up ticks) and process memory of each `spade_viewer <scene> cpu --trajectory` run | No: per-run numbers |
| `viewer/<scene>/t0.png` … `t8.png` | v1's rendering of the scene: the first drawn frame, then 1 s, 3 s and 8 s of wall time after it | No: wall-clock moments, not ticks |
| `sandbox/<scene>/paused.png`, `play2.png` … `play10.png` | v1 `Sandbox`: the paused first frame, then 2 s, 5 s and 10 s of wall time after motion was first seen | No: the scene is unseeded |
| `*/<scene>/capture.json` | Executable hash, timings, capture method per frame, play attempts, memory samples, the tool's printed FPS | Timing varies |
| `*/<scene>/stdout.txt` | The tool's own output, as printed | — |

## How to read the performance numbers

- **Physics step time** (viewer scenes): `step_ns_median` and `step_ns_p90` in `performance.json`, timed around each `step(1)` after 100 warm-up ticks, CPU backend, Release.
- **Memory:** both private bytes (v1's `Mem` metric) and working set (the sandbox's metric) are recorded, so either can be compared.
- **v1's printed FPS is not a frame rate below about 20.** `Engine::UpdateStatistics` clamps dt at 0.05 s before it counts frames (`src/Core/Engine.cpp:604`). For the `Sandbox`, which prints one line per frame, `frames_per_wall_second` in `capture.json` is the real rate.
- **Render time is not comparable** with any successor. The CPU rasterizer is slower than hardware GL by design (`SL14b`).

## The scenes

All eight viewer scenes step at 4 ms with 4 substeps (250 ticks/s) on the CPU backend. Plain bodies have a mass of 1 kg.

| Scene | What it exercises | Recorded behaviour |
|---|---|---|
| drop | 6 bodies dropped from rest onto a plane; restitution 0.35, friction 0.5; seed 1 | Bounce, then settle |
| bounce | 4 worlds, one ball each from 6 m; restitution 0, 0.25, 0.5 and 0.75; seeds 1–4 | One restitution per lane, batched |
| shower | 1000 bodies on a jittered lattice into an open tub (a union of 5 boxes); seed 7 | Dense dynamic contact |
| gate | 5 balls lobbed through a torus ring between two box posts; seed 3 | SDF torus and box contact |
| hover | 1 quadrotor in trim at 3 m; seed 11; no turbulence | Holds station |
| wind | hover with moderate Dryden turbulence; seed 23 | Climbs to a peak near 8 m at about 3.5 s, then descends; attitude unchanged |
| flight | 1 quadrotor through the gate on a scripted command profile; seed 23 | Crosses the gate plane at about 7.86 s |
| swarm | 4 worlds, one quadrotor each, in trim; seeds 101–104 | Four lanes hold station |
| `Sandbox` fluid | 50,000 SPH particles in a 10 m cube; unseeded | Capability demonstration only (`SL14c`) |
| `Sandbox` spheres | 50,000 spheres, bounciness 0.3, grid collision; unseeded | Capability demonstration only |
| `Sandbox` cubes | The spheres scene drawn with cube meshes | Capability demonstration only |

## The asymmetries `SL14b` names

- **Functional:** the eight viewer scenes compare on bit-identical trajectories (the digests here). The three `Sandbox` scenes have no trajectory; a successor shows the same systems running stably.
- **Visual:** judged on geometry, motion and materials, never a pixel diff. v1 draws through OpenGL; successors draw through Spade's renderers.
- **Performance:** physics step time and memory for the viewer scenes; fresh numbers only for the `Sandbox` scenes.
