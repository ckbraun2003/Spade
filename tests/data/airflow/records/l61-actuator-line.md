# L61 V2, one rotor hovering in a CPU grid box: the actuator line against the BEMT -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). The bands are Test/Docs',
`docs/design/test-docs/2026-11-11-l61-actuator-line-bands.md`, fixed before the first run. The measured lines are the block
`AirflowLineHover.V2OneRotorInAGridBox` prints (tests/test_airflow_line_v2.cpp, opt-in SPADE_V2_RDX); the polar is the estimated one of
engine/physics/airflow/blade.hpp, uncalibrated (V1a's static point stood). The comparator is the BEMT with Prandtl's tip loss on the same stations
and polar. R/dx 38 is the GPU's (P4c) and is not here.

```text
=== validation record: L61 V2 one rotor hovering, the actuator line at R/dx 19, D1's rotor ===
commit: b984d496 (physics/line-b, the V2 test)
date: 2026-11-11
path: CPU reference, MSVC, release (engine/physics/airflow/line.cpp, fluid_step, estimated polar, n_sub 8)
config: triblade_6inch_scaled stations at R 0.0762 m, 3 blades, 801 rad/s, rho 1.225; box 100x158x100 = 1.58 M cells, dx 4.01 mm, all faces open, still air; 1188 steps of 0.381 ms (418 s wall); windows [20,30] and [30,40] R/v_h, v_h 6.737 m/s from the BEMT
measure: line thrust = 2.0526 N (windows 2.0529, 2.0526 N; differ 0.02%: settled)
reference: BEMT with Prandtl tip loss (tests/bemt_reference.hpp) 2.0285 N, the line +1.2%
tolerance: within 10% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power (torque x omega) = 17.183 W
reference: BEMT with Prandtl tip loss 19.771 W, the line -13.1%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: momentum flux through the plane 1.5 R below, over the line's own thrust = 1.052
reference: steady momentum theory, centre 1 (rho sum w^2 dA of the downward cells; Test/Docs bands note)
tolerance: within 10% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: figure of merit T^1.5 / (sqrt(2 rho A) P) = 0.810
reference: none (report only; UIUC static value 0.573)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L61 V2 one rotor hovering, the actuator line at R/dx 25, D1's rotor ===
commit: b984d496 (physics/line-b, the V2 test)
date: 2026-11-11
path: CPU reference, MSVC, release (engine/physics/airflow/line.cpp, fluid_step, estimated polar, n_sub 8)
config: triblade_6inch_scaled stations at R 0.0762 m, 3 blades, 801 rad/s, rho 1.225; box 132x207x132 = 3.61 M cells, dx 3.05 mm, all faces open, still air; 1563 steps of 0.29 ms (1143 s wall); windows [20,30] and [30,40] R/v_h, v_h 6.737 m/s from the BEMT
measure: line thrust = 2.0612 N (windows 2.0601, 2.0612 N; differ 0.05%: settled)
reference: BEMT with Prandtl tip loss (tests/bemt_reference.hpp) 2.0285 N, the line +1.6%
tolerance: within 10% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power (torque x omega) = 16.910 W
reference: BEMT with Prandtl tip loss 19.771 W, the line -14.5%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: momentum flux through the plane 1.5 R below, over the line's own thrust = 1.062
reference: steady momentum theory, centre 1 (rho sum w^2 dA of the downward cells; Test/Docs bands note)
tolerance: within 10% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: figure of merit T^1.5 / (sqrt(2 rho A) P) = 0.828
reference: none (report only; UIUC static value 0.573)
tolerance: none (report only)
verdict: report only
```

## 2026-11-11: what these two blocks say

Physics. R/dx 19 and 25 side by side: thrust +1.2% and +1.6% of the BEMT; power -13.1% and -14.5%; flux ratio 1.052 and 1.062; FM 0.810 and 0.828.
Thrust and flux are inside their bands and nearly flat in resolution. POWER DOES NOT CLOSE WITH RESOLUTION: the line's torque falls from -13.1% to
-14.5% of the BEMT as the grid is refined, towards the 15% edge, and R/dx 38 may pass it. That is a finding, not a band change: the line has less
drag-side load than the BEMT as its kernel narrows (eps = 2 dx), the thrust held by lift. The R/dx rule's own test (R/dx 19 within 5% of R/dx 38 in
thrust, 7.5% in power) cannot be read until the GPU's R/dx 38: from these two the power already moves 1.4 points per 6 cells of R/dx.
