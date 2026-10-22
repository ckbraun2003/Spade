# D1, the hover preview -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; checks and rows in `docs/design/test-docs/2026-10-21-d1-hover-checks.md`). The reference quad held level at hover, solid, in a 3 m/s wind, run through `LiveRun` at 4 mm. Physics writes this record (the lead's ruling, 2026-10-21).

```text
=== validation record: D1 hover preview, the reference quad in a 3 m/s wind ===
commit: 1f5fb1a (the frozen flow_movie build; one queue job, 3000 s chunks, 9.4 h, exit 0 at 2026-10-22 12:01Z)
date: 2026-10-22
path: CPU reference, MSVC, release (tools flow_movie --video d1-hover --seconds 0.55, LiveRun)
config: 224x160x168 cells at dx 4 mm (6.02 M), R/dx 19, nu 1.5e-5 m2/s, wind 3 m/s along +x, drone held at (0.000, -0.006, 0.000); 17 frames, 1492 steps, t 0.567 s; D1-3's window is frames 9 to 14, t 0.300 to 0.467 s; run line: solids: 10 parts, 10 walls, 0 sheets; unconverged steps from Rendering's convergence.csv.recovered (one row per frame; its last residual is the last step's, not the largest)
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
