# D1, the hover preview -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; checks and rows in `docs/design/test-docs/2026-10-21-d1-hover-checks.md`). The reference quad held level at hover, solid, in a 3 m/s wind, run through `LiveRun` at 4 mm. Physics writes this record (the lead's ruling, 2026-10-21).

```text
=== validation record: D1 hover preview, the reference quad in a 3 m/s wind ===
commit: 1f5fb1a (the frozen flow_movie build; one queue job, 3000 s chunks, 9.4 h, exit 0 at 2026-10-22 12:01Z)
date: 2026-10-22
path: CPU reference, MSVC, release (tools flow_movie --video d1-hover --seconds 0.55, LiveRun)
config: 240x177x168 cells at dx 4 mm (7.14 M; 0.96 x 0.708 x 0.672 m, kD1Grid, a box from (-0.480, -0.354, -0.336)), R/dx 19, rho 1.225 kg/m3, nu 1.4607e-5 m2/s (the case's ISA default at 15 C), wind 3 m/s with the air moving along -x (hover_in_wind, heading 0), no floor, drone held at (0.000, -0.006, 0.000), rotors as actuator discs at hover from the start (throttle 0.165, 801 rad/s); h 0.38 ms, 17 frames, 1492 steps, t 0.567 s; D1-3's window is frames 9 to 14, t 0.300 to 0.467 s; run line: solids: 10 parts, 10 walls, 0 sheets; unconverged steps from Rendering's convergence.csv.recovered (one row per frame; its last residual is the last step's, not the largest)
measure: parts in the run line = 10
reference: the library quad's rigid parts, 10 (body, four arms, four motors, battery; tests/test_vehicles_airflow_solids.cpp, TheReferenceQuadAtHoverIsTenCleanWallsAt4Millimetres)
tolerance: exact (docs/design/test-docs/2026-10-21-d1-hover-checks.md, D1-1, fixed before the run)
verdict: within
measure: walls in the run line = 10
reference: every part a wall, 10 (D1-1)
tolerance: exact (docs/design/test-docs/2026-10-21-d1-hover-checks.md, D1-1, fixed before the run)
verdict: within
measure: sheets in the run line = 0
reference: no sheet at 4 mm, 0 (D1-1)
tolerance: exact (docs/design/test-docs/2026-10-21-d1-hover-checks.md, D1-1, fixed before the run)
verdict: within
measure: safeguard advisories after the counts in the run line (leaks at an edge, no closed face) = 0
reference: none (D1-2)
tolerance: exact (docs/design/test-docs/2026-10-21-d1-hover-checks.md, D1-2, fixed before the run)
verdict: within
measure: steps announced unconverged (above 1e-4), the whole run = 1
reference: none (report only; the run announces each such step)
tolerance: none (report only)
verdict: report only
measure: steps announced unconverged before D1-3's window (t under 0.300 s) = 1
reference: none (report only); it is step 1 from rest, in frame 1
tolerance: none (report only)
verdict: report only
measure: steps announced unconverged within D1-3's window (frames 9 to 14) = 0
reference: none (report only); the file's running count did not grow across frames 9 to 14
tolerance: none (report only)
verdict: report only
measure: the largest last residual among the window's frames 9 to 14 (frame 11) = 5.04e-07
reference: none (report only); the rule's bar within the window is 1e-3
tolerance: none (report only)
verdict: report only
measure: the largest last residual over all 17 frames (frame 3) = 2.224e-06
reference: none (report only); the smallest is 1.645e-07 (frame 17)
tolerance: none (report only)
verdict: report only
```

## 2026-10-22: what this record does not hold

Physics, from Rendering's recovered log and the lead's words, 2026-10-22. This is prose, not a record: readers read only the fenced block above.

- **Not measured:** D1-3 (the downwash at 2R below each hub). The arm-top C_p sign. D1-5 (`no_data` against the solid cells). In each case: not measured: the cached frames were deleted before the samples were taken (2026-10-22); to be re-run on the GPU solver. None is a pass or a fail. The downwash magnitude, report only, has no value either.
- **D1-4:** not judged here. The film's caption is judged from the posted video's caption by Test/Docs. The editor's legend is judged in the editor's own check.
- **Unconverged steps, the limit:** `convergence.csv` keeps each frame's last residual, not its largest. So the largest residual in the window is not known. The count stands instead: a step over 1e-4 is announced and counted, and the count did not grow across frames 9 to 14. The window's last residuals are 3.314e-07, 3.099e-07, 5.040e-07, 2.192e-07, 2.641e-07 and 3.662e-07. Every frame ends between 1.645e-07 and 2.224e-06.
- **Step 1:** the one announced step is step 1 from rest, in frame 1. Its own residual is not in this run's file. The timing run (`2ddb2b9`) measured it at 1.26e-04 at the CG cap of 32.
- **Iterations per step:** not recorded in this run (`LiveRun` did not expose them).
- **Provenance:** the file was recovered from the run's log after the render removed the cache (`convergence.csv.recovered`, with its README beside it). The posted video is `Videos/Research/Spade/2026-10-22_1202_d1-hover-parts-in-flow.mp4`.
- **The run line:** the counts are as burned into every view. The advisory row reads "nothing follows the counts" in that line.
- **The case run, against the spec's §1** (Rendering, from `live_case.cpp`'s `d1_live_case` on 1f5fb1a and the run's log): the grid is 240 x 177 x 168, not 224 x 160 x 168. LiveRun holds the drone at the grid's centre, so `kD1Grid` is the smallest box about it that keeps each of the spec's clearances on both sides. `nu` is 1.4607e-5 m2/s, the case's ISA default, not 1.5e-5. The air moves along -x, the mirror of the spec's +x, which the plus layout makes equivalent. None of these bears on the measures above, which are the run line's counts and the solver's own residuals.

## 2026-10-27: the disc this record was measured with

measured with the 3×16 disc, whose concentrated forcing drove a non-physical 85-150 m/s jet near each disc (triage 2, 2026-10-27); superseded by the dense disc; re-measured on the GPU at B7b

## 2026-11-03: B7 acceptance on the device, with the solids (B8)

GpuFluid, the whole step on the device of record (RTX 3060 Ti, driver 572.83), against the CPU's run of the same box over 32 steps: TD-14 as amended, bit-equal. The step time is report only (the lead, 2026-11-03). This is the D1 block the gauge work is measured against: 293.1 ms a step against Cameron's bound of 190 ms with the bits kept. The film's 0.50 s ran too, report only.

```text
=== validation record: B7 acceptance, D1 box 240x177x168, GpuFluid (G3) against the CPU ===
commit: dfefd3fc (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8-d1-dfefd3fc)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: D1: 240x177x168 at dx 4 mm (kD1Grid, tests/d1_case.hpp), wind (-3.0, -0.0, 0.0) m/s; the reference quad's rigid parts as solids, LiveRun's own (solids: 10 parts, 10 walls, 0 sheets); held at hover, its 4 rotors as actuator discs at their commanded hover speed, 801 rad/s, 5.158 N in all; rho 1.225, nu 1.4607e-05, h 0.38 ms, CG floor 8, cap 32; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; 0.50 s of the box's time in 1316 steps, published at every 1/30 s
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 917091424
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 293.1 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
measure: the wall time for 0.50 s of the box's time (1316 steps), publishing at the film's frame times = 390.17 s
reference: the plan's estimate, about 2 minutes at the D1 box (B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 20.08 m/s against 54.97 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 20.08 m/s against 54.97 m/s; its mean speed at step 32 = 0.402942 m/s, the CPU's 0.402942 m/s (within 10% + 1e-04 m/s)`

## 2026-11-03: the closed-region gauge over each region's own tiles (Rendering)

GpuFluid with the CG's gauge (`airflow_region_mean`) over each closed region's own tiles, packed once at set-up, the whole step on the device of record (RTX 3060 Ti, driver 572.83), against the CPU's run of the same box over 32 steps: TD-14 as amended, bit-equal. The step time is report only: 138.4 ms a step, against B8's 293.1 ms above and Cameron's bound of 190 ms with the bits kept (2026-11-03). The film's 0.50 s took 187.03 s (B8: 390.17 s), report only.

```text
=== validation record: B7 acceptance, D1 box 240x177x168, GpuFluid (G3) against the CPU ===
commit: e3329a13 (rendering/gauge-tiles; queue ticket spade-rendering-d1rec-gauge-e3329a13)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: D1: 240x177x168 at dx 4 mm (kD1Grid, tests/d1_case.hpp), wind (-3.0, -0.0, 0.0) m/s; the reference quad's rigid parts as solids, LiveRun's own (solids: 10 parts, 10 walls, 0 sheets); held at hover, its 4 rotors as actuator discs at their commanded hover speed, 801 rad/s, 5.158 N in all; rho 1.225, nu 1.4607e-05, h 0.38 ms, CG floor 8, cap 32; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; 0.50 s of the box's time in 1316 steps, published at every 1/30 s
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 917091424
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 138.4 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
measure: the wall time for 0.50 s of the box's time (1316 steps), publishing at the film's frame times = 187.03 s
reference: the plan's estimate, about 2 minutes at the D1 box (B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 20.08 m/s against 54.97 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 20.08 m/s against 54.97 m/s; its mean speed at step 32 = 0.402942 m/s, the CPU's 0.402942 m/s (within 10% + 1e-04 m/s)`

## 2026-11-04: L13, the aperture floor (Physics)

build_solids() closes every wall face whose ramp leaves under 5% open (kApertureFloor; the lead's L13). The device against the CPU, 32 steps, D1 with its solids: still bit-equal on the device of record. The step time is report only.

```text
=== validation record: B7 acceptance, D1 box 240x177x168, GpuFluid (G3) against the CPU ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: D1: 240x177x168 at dx 4 mm (kD1Grid, tests/d1_case.hpp), wind (-3.0, -0.0, 0.0) m/s; the reference quad's rigid parts as solids, LiveRun's own (solids: 10 parts, 10 walls, 0 sheets); held at hover, its 4 rotors as actuator discs at their commanded hover speed, 801 rad/s, 5.158 N in all; rho 1.225, nu 1.4607e-05, h 0.38 ms, CG floor 8, cap 32; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; 0.50 s of the box's time in 1316 steps, published at every 1/30 s
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 917091424
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 125.6 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
measure: the wall time for 0.50 s of the box's time (1316 steps), publishing at the film's frame times = 177.92 s
reference: the plan's estimate, about 2 minutes at the D1 box (B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them:

- `the CPU's largest |v| at step 32: 16.4935 m/s`
- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 20.06 m/s against 54.97 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 20.06 m/s against 54.97 m/s; its mean speed at step 32 = 0.402855 m/s, the CPU's 0.402855 m/s (within 10% + 1e-04 m/s)`
