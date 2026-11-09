# The Run air Loads row's hover download: the bound, fixed before the test's first run

**Owner:** Test/Docs. **Status:** written 2026-11-09 from Physics' measurement (merged 832abccb), before the headless Loads test (Interface's Loads row, after stage 7 E) exists or has run.
**For:** the headless test that the held reference quad at hover, at the Detailed level on the CPU, shows a download of the right sign under an upper bound.
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the measurement and the method. It is not loosened after a run without a lead decision.

## The measurement

`tests/test_hover_download.cpp` (physics/hover-dl-progress, e7e3f7ad), `SPADE_HOVER_DOWNLOAD=2.0`, 5373 s. The pressure force on all solids together, grid axes (Y up), CPU reference.

- **The configuration:** the actuator disc, 104³ cells at dx 8 mm, h 1 ms (the Detailed level, CFL up to 2), hover throttle, still air, discs on. 2000 steps over 2.00 s, the first 0.5 s dropped, three windows of 500 steps.
- **The solids:** at the 8 mm cell only 2 parts and 2 walls voxelise. The arms are drawn as a wire and the motors leak. So the load below is on those four, **not on the quad's ten rigid parts**.
- **F_y, the window means:** −0.30847, −0.31167, −0.30800 N. Their mean is −0.30938 N, their sd 0.00200 N (0.65%).
- **Steps:** sd 0.00657 N, range −0.33235 to −0.29533 N. F_x +0.0004 N and F_z −0.0003 N, nothing sideways.
- **Settled:** by 0.4 to 0.5 s (the 0.1 s samples go +0.007, −0.228, −0.260, −0.287, −0.296 N), with no drift across the windows.

## The bound

**The Loads row's 0.5 s mean of F_y, for a window that starts at 0.5 s or later, with Y up, is a download of 0.29 N ≤ |F_y| ≤ 0.32 N** (F_y negative). Both edges come from the same terms. The floor is what a one-sided bound lacks: a solver that has lost most of its load (F_y = −0.01 N) is "< 0 and ≤ 0.32" and must still fail.

The bound is on a window mean, not on a step. A single step can reach 0.332 N, so a step is never tested against it.

| Term | Value | Built from |
|---|---|---|
| The measured mean | 0.30938 N | the three window means |
| A window other than the three measured | 0.00994 N | the 95% prediction half-width for one new window from three: t(0.975, 2) = 4.303 × sd 0.00200 × √(1 + 1/3). Three is few, so the whole t-interval is taken, not 2 sd |
| The upper edge: the mean plus that term | 0.31932 N | rounded up to two figures: **0.32 N** |
| The lower edge: the mean minus that term | 0.29944 N | rounded down to two figures: **0.29 N** |
| Platform, thread count | 0 | the project's rule is that the same inputs give the same results, and the run used 8 threads. Not measured at another count: the test's first run records its first window beside −0.30847 N |

- **The sign** needs no margin of its own. The mean sits 155 window sd below zero (0.30938 / 0.00200), and a flipped sign would be an upload.
- **The floor** (the lead's ruling, 2026-11-09) is the lower edge above. The sign is part of the band: a download, F_y < 0.
- **Start-up is excluded, not banded.** A window that includes the first 0.5 s reads lower (the 0.1 s samples start near zero, and reach −0.296 N only by 0.5 s), so it can fall under the floor. It is not the measured quantity. The test drops the first 0.5 s: a 1.0 s run, the mean over steps 500 to 999.

## What the bound does not cover

- **A different configuration needs a new measurement,** not a looser bound: another grid or dx, h, CFL, a change to the solver's pressure or wall treatment, the solid set (ten solids instead of four), or the throttle. Interface's Detailed sentence says the load is within about 40% of the same run at CFL 0.5 (L13), so a retuned level moves it by far more than the 0.29 to 0.32 N band allows.
- **The compiler.** Measured on the MSVC build. A gcc run is a second measurement (an implementation band, `01-verification.md`), not covered here.
- **The label.** The row, the test's failure message and its record carry "the load on the 2 parts and 2 walls that voxelise at 8 mm", never "the quad's load".
- **The axes.** F_y here is in grid axes, Y up, a download negative. A Loads row that reports a download as positive tests 0.29 N ≤ |F_y| ≤ 0.32 N and "download" in place of F_y < 0.
- **The cost.** 5373 s for 2000 steps on a shared box, so about 2700 s for the 1000 steps the test needs. It cannot join either gate (the debug ctest limit is 60 s). It runs opt-in, env-gated like the hover-download row: `SPADE_LOADS_BAND=1` runs it, and without it the test skips, the skip message naming the variable (the lead's ruling, 2026-11-09).
- **The first run** records its first window (steps 500 to 999) beside Physics' −0.30847 N, with the thread count it ran at. A thread count other than 8 that moves the window is a finding, not a reason to widen the band.
