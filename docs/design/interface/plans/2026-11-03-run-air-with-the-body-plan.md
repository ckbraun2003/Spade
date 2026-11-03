# Run air with the drone's body as a solid on the GPU: Interface's plan (item 1)

Interface, 2026-11-03, for the lead. Item 1 of `2026-11-03-geometry-and-air-items.md`, Cameron's next visible result (backlog `145386d9`). It builds on Physics' P-1 note (`.agents/physics-p1-note.md`), which the lead ruled on 2026-11-03: N15 is amended for the live run only. Nothing is built yet. Building starts when the lead OKs this plan and Physics' `refused_part` seam is on master.

**Pending:** Rendering's device numbers at 104³ and 125³ with the body: real-time fraction, memory, start-up. They set the cell cap in §3 (128 until then). Physics' ruling on the step.

## What the user sees

In the editor, the user places the Phantom, adds its airflow region and presses Run air. The air runs on the GPU at 12 mm (104³), with the body, and the motors where they resolve, as solids in the wake. The band names every part left out of the air and why. With no device, the run is discs only on the CPU's coarse grid, and the band says so.

## 1. The loop (LiveRun::start, `engine/analysis/live_run.cpp`)

It goes where `AirflowSolids::parts()` and `set_solids()` are called now (lines 362 to 405).
1. Classify each rigid posed part at the grid's dx with `classify_part()`. A part classed thin (a sheet), refused (a wire or a fastener) or porous is drawn only, with that reason. The user's own drawn-only mark (N15) is kept, with its own reason.
2. Call `set_solids()` with the rest. If it refuses with `SolidField::refused_part` set (a leak, or a part exposed inside another), that part is drawn only with that reason, and `set_solids()` runs again without it. The loop stops when a build is accepted or no part is left: at most as many builds as there are parts. Each build takes 120 ms at 104³ (Physics' measurement); the Phantom's worst case is about 0.7 s, once, at Run air's start.
3. Any other refusal still refuses the run, as now: a non-finite sampler, or a part outside the grid.
4. **No part left:** the run is discs only. That is plan A at the case's n_v: no solids, today's bits.
- The kept set follows from the case alone, so the same placement always gives the same set. `LiveRun::restore()` goes through `start()` (`live_run.cpp:829`), so a restore rebuilds the same set and needs no format change.
- **The imported-part refusal flips.** Today an imported part the grid can't resolve refuses the run with "or mark part '…' visual-only" (`live_run.cpp:376-385`, `test_analysis_live_run.cpp:590`). It becomes drawn only, named in the label, as P-1 rules.
- Offline runs and films don't use LiveRun, so they keep refusing.

**Needs from Physics, typed so the loop never reads the refusal's words:** the refused part's index, and its reason (leaks, or exposed inside another part). For an exposed part, also the index of the part it lies inside. P-1 names `SolidField::refused_part`; the reason and the containing part are what I add to that ask.

## 2. The label (`LiveRun::run_label()`)

The segment " · N parts drawn only: 'a', 'b'" becomes the drawn-only segment below. Parts are grouped by reason, in the order the parts are declared, and consecutive indexed names are joined, so `arm[0]` to `arm[3]` read as `arm[0-3]`:

`· drawn only at 12 mm: arm[0-3], esc (thinner than 2 cells); battery (inside 'body', exposed); motor[1] (leaks)`

- **The reasons:** "thinner than 2 cells" (a sheet), "a wire" (a wire or a fastener), "porous", "leaks", "inside '<part>', exposed", "marked drawn only".
- **With no part left:** `· discs only: every part drawn only at 39 mm: body, arm[0-3], … (thinner than 2 cells)`.
- The solids' run line (`solids.run_line`) stays as it is when any part is in the air.
- The band still elides at its end when it doesn't fit (`band_shown()`). The full label stays in the run.

## 3. The default cell (`AddAirflowRegion`, `sandbox/editor_scene_edits.hpp:306`)

- **The box is today's:** 32 × floor-to-the-mm(R/3). It doesn't change, so the wake has the room it has now.
- **The cell:** dx = floor-to-the-mm(min(R/3, F/4)), where F is the largest min_feature among the rigid parts. Then n = ceil(box/dx), in place of `kAirCells`.
- **The cap (ruled): n ≤ n_max,** 128 until Rendering's numbers set it. With no cap, the Crazyflie would get a 1 mm cell, 224³ (11.2 M cells). Past the cap, dx = box/n_max, and the loop draws whatever the coarser cell can't resolve.

The library drones under the rule (computed from the model files; their boxes as now):

| Drone | R | F | Now | Under the rule |
|---|---|---|---|---|
| Phantom | 119 mm | 50 mm | 32³, 39 mm | 104³, 12 mm, R/dx 10 |
| Iris+ | 122 mm | 50 mm | 32³, 40 mm | 107³, 12 mm |
| reference quad, 6-inch, competition | 76 mm | 35 mm | 32³, 25 mm | 100³, 8 mm |
| 5-inch FPV | 64 mm | 30 mm | 32³, 21 mm | 96³, 7 mm |
| Crazyflie | 23 mm | 7 mm | 32³, 7 mm | 128³, 1.75 mm (the cap; 224³ at 1 mm without it) |

- **The reference quad moves too** (ruled). It has 10 rigid parts, F = 35 mm. Test/Docs' C8q and the GPU tests use its case. They pin today's grid (§5), so their bits hold.
- A drone with no rigid part keeps R/3 at 32³.
- Saved scenes keep their saved grids. A region saved at 39 mm runs as today: every Phantom part is thin there, so it is discs only, and now it says so.

## 4. The CPU fallback (the lead's ruling, 2026-11-03)

- **No device, or the device refuses the case:** the run is discs only on the CPU, on R/3's grid over the same box (32³ for every library drone).
- **Why that grid:** the CPU doesn't run discs at 104³ live either, since that is about 1 s a step (an estimate: M0's 28.7 ms at 32³ times 34 for the cell count).
- **The band's backend:** `CPU: <why>; discs only, the body needs the GPU` when the drone has a rigid part not marked drawn only. Otherwise it stays `CPU: <why>`. For example: `air at 1/43 speed · actuator disc · CPU: no Vulkan device; discs only, the body needs the GPU · 32³, dx 39 mm, R/dx 3`. `band_floor()` stays "… · CPU".
- **The wiring:**
  - `air_model()` keeps the parts, and `air_case()` resolves the region's grid: the body case.
  - A new `air_discs_case()` resolves the fallback grid with the parts cleared: the discs case.
  - The `AirStarter` takes both cases.
  - `gpu_air_starter()` tries the device with the body case and starts the discs case on the CPU when it can't. That needs `start_live_run()` (`spike/airflow/live_fluid.hpp:116`, Rendering's) to take an optional fallback case for its CPU start: a few lines, for Rendering to OK and review.
  - `cpu_air_starter()` starts the discs case.
  - When the two cases are the same (the region already at R/3 or coarser, with no part left), only one run is started.

## 5. Tests

RED first in each slice.

**The engine slice, the CPU twins at a small grid** (`test_analysis_live_run.cpp`):
1. The Phantom at 12 mm on 48³ (a 0.576 m box, which holds its 0.49 m span).
   - The run starts, the body is solid (solid cells > 0), and the label names each drawn-only part with its reason.
   - Each part's fate matches classify_part(), and the safeguard's drops match a hand-run build-and-drop in the same order.
   - Two starts give the same set and the same label.
   - One frame advances with no error.
2. The Phantom at 39 mm: every part is thin, so the run is discs only and says so. Its frames equal today's run with the parts cleared, to the bit (both have no solids).
3. The flip: the imported part at line 590 is drawn only and named, not refused. The test's assertions change, and I name it in READY.
4. A part outside the grid and a non-finite sampler still refuse (the existing tests, unchanged).
5. A restore of the 48³ body run has the same kept set and continues to the bit.

**The editor slice** (`test_editor_scene_edits.cpp`, `test_editor_airflow.cpp`):
1. AddAirflowRegion's cell: the Phantom 104³ at 12 mm, the reference quad 100³ at 8 mm, the Crazyflie at the cap, and a drone with no rigid part at R/3 on 32³.
2. The CPU fallback (`cpu_air_starter()`): the backend's words, and the run equal to a one-thread LiveRun of the discs case on its 32³, to the bit.
3. `ThePlacedPhantomsAirStartsAndPublishes`: updated to the new default and its CPU fallback.

**On the device** (`test_gpu_editor_airflow.cpp`):
1. `ThePlacedPhantomsAirOnTheDeviceIsTheCpusToTheBit` grows: the body on the device at 48³ and 12 mm. Its published halves after a frame equal a one-thread CPU LiveRun of the same body case, to the bit, and the label and solid cells match. Opt-in if it runs past 30 s in release.
2. The reference-quad tests pin today's grid (`with_library_drone()` takes the cells). Every reference-quad part is thin at 25 mm, so they stay discs only and keep their bits. Only their labels change.
3. The full 104³ run's frame time and memory are Rendering's measurement. There's no new test for them.

**The live smoke:** editor.airflow places the Phantom and adds its region (`live_tour.cpp:1393`), so it moves to 104³ with the body on the GPU. Its frame rate and frame count move, and that is the visible check. The tour's airflow chapter is item 3 and doesn't change here.

**Gates:** build, ui, CPU, smoke, GPU, gcc. No LEG: no file format changes. Debug at the batch gate, with the new CPU twins timed in debug first (48³ with solids).

## 6. Slices

1. **The engine** (§1 and §2): the loop and the label, after Physics' seam. Physics reviews.
2. **The editor** (§3 and §4): the default cell, `air_model()` keeping the parts, the two cases, the fallback, and the band. Rendering reviews the starter and `start_live_run()`.

Each slice is RED then GREEN on its own `interface/*` branch.

## The lead's rulings (2026-11-03: plan OK, the engine slice first after Physics' seam)

1. **The cap.** n_max is set from Rendering's 104³ and 125³ numbers, and is 128 until they land. A drone that hits the cap takes dx = box/n_max, so the Crazyflie gets 1.75 mm on 128³. The loop then draws, and names, whatever that cell can't hold, so the Crazyflie flies as discs or with its body only.
2. **The reference quad's default moves to 100³ at 8 mm.** One rule holds for every drone. The tests that pin today's grid keep their bits, and the READY names the moved defaults.
3. **The fallback.** Discs only on R/3's grid over the same box, as §4 plans. Granted: `start_live_run()` takes an optional fallback case. Rendering reviews it in the editor slice.
4. **Physics' seam** carries the typed reason (leaks or exposed) and, for an exposed part, the part it lies inside, beside `refused_part`.
5. **The step.** Physics rules whether 1 ms holds at 7 to 12 mm beside walls, or whether dt follows dx. At 12 mm, the wake's fastest cells cross about 1 to 1.5 cells a step (11.85 m/s, poster 02). The editor slice takes dt from a function of dx, which today returns 1 ms (`kAirStepS`), so either ruling is a one-line change.
