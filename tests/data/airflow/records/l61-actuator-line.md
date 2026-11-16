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

```text
=== validation record: L61 V2 one rotor hovering, R/dx 19, the air READ with a 1 dx kernel (negative result), D1's rotor ===
commit: c56702c9 (physics/line-diag; the sample_eps_cells option, reverted before merge: it is not in the model)
date: 2026-11-11
path: CPU reference, MSVC, release (line.cpp with LineRotor::sample_eps_cells = 1; the force still spread with eps 2 dx; n_sub 8)
config: as the R/dx 19 block above (triblade_6inch_scaled stations, R 0.0762 m, 801 rad/s, box 100x158x100, 1188 steps), only the sample kernel narrower
measure: line thrust = 2.0525 N (windows 2.0521, 2.0525 N; differ 0.02%: settled)
reference: BEMT with Prandtl tip loss 2.0285 N, the line +1.2%
tolerance: within 10% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power (torque x omega) = 16.977 W
reference: BEMT with Prandtl tip loss 19.771 W, the line -14.1% (the default 2 dx sample: -13.1%)
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: figure of merit T^1.5 / (sqrt(2 rho A) P) = 0.819
reference: none (report only; the default sample 0.810, the BEMT 0.69, UIUC static 0.573)
tolerance: none (report only)
verdict: report only
```

## 2026-11-11: what these two blocks say

Physics. R/dx 19 and 25 side by side: thrust +1.2% and +1.6% of the BEMT; power -13.1% and -14.5%; flux ratio 1.052 and 1.062; FM 0.810 and 0.828.
Thrust and flux are inside their bands and nearly flat in resolution. POWER DOES NOT CLOSE WITH RESOLUTION: the line's torque falls from -13.1% to
-14.5% of the BEMT as the grid is refined, towards the 15% edge, and R/dx 38 may pass it. That is a finding, not a band change: the line has less
drag-side load than the BEMT as its kernel narrows (eps = 2 dx), the thrust held by lift. The R/dx rule's own test (R/dx 19 within 5% of R/dx 38 in
thrust, 7.5% in power) cannot be read until the GPU's R/dx 38: from these two the power already moves 1.4 points per 6 cells of R/dx.

## 2026-11-11: the narrower sample is a null result (do not retry it)
Physics. Reading the air with a 1 dx kernel in place of 2 dx (the force still spread at 2 dx) was tried for the power gap: a frozen-field estimate had
predicted +5% induced power. The self-consistent R/dx 19 run gave induced 12.067 W (12.078 W at 2 dx), profile 4.718 W, power -14.1% of the BEMT
(-13.1% at 2 dx), thrust +1.2% (same), flux 1.057, FM 0.819 (0.810), and the tip's axial speed at r/R 0.975 stayed 0.68 m/s (0.69). A higher sampled
downwash lowers the angle of attack and the loads give it back; the tip's low inflow is the field's tip vortex, which a narrower average does not move.
The option was removed before merge. (The tip correction, L72, was tried next and also failed to explain the gap: see the 2026-11-16 paragraph.)

```text
=== validation record: L61 V2 the line's kernel width, R/dx 19 and 25, tip correction off, D1's rotor ===
commit: 12751cd1 (physics/line-diag; LineRotor::eps_cells, the kernel's width in cells, default 2; SPADE_V2_EPS_X100)
date: 2026-11-16
path: CPU reference, MSVC, release (the force spread and the air read with the same Gaussian, eps as listed; n_sub 8 except where stated)
config: as the R/dx 19 block above (triblade_6inch_scaled stations, R 0.0762 m, 801 rad/s); R/dx 25: box 132x207x132, h 0.29 ms, 1563 steps
measure: line power, R/dx 19, eps 6.1 mm (1.52 cells) = 16.837 W (thrust +2.2%, FM 0.838)
reference: BEMT with Prandtl tip loss 19.771 W, the line -14.8%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 25, eps 6.1 mm (2.0 cells, the default there) = 16.910 W (thrust +1.6%, FM 0.828)
reference: BEMT with Prandtl tip loss 19.771 W, the line -14.5%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 19, eps 8.0 mm (2.0 cells, the default there) = 17.183 W (thrust +1.2%, FM 0.810); with n_sub 16: 17.187 W
reference: BEMT with Prandtl tip loss 19.771 W, the line -13.1%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 25, eps 8.0 mm (2.62 cells) = 17.280 W (thrust +0.8%, FM 0.800)
reference: BEMT with Prandtl tip loss 19.771 W, the line -12.6%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 19, eps 10.0 mm (2.5 cells) = 17.355 W (thrust +0.5%, FM 0.793)
reference: BEMT with Prandtl tip loss 19.771 W, the line -12.2%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 19, eps 12.0 mm (3.0 cells, the support's limit) = 17.466 W (thrust +0.5%, FM 0.788)
reference: BEMT with Prandtl tip loss 19.771 W, the line -11.7%
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
```

```text
=== validation record: L61 V2 the tip correction (Martinez-Tossas and Meneveau 2019, explicit filtered lifting line), R/dx 19 and 25 (negative result) ===
commit: 334d28ef, tag physics/l72-tip-correction-334d28ef (the code lives ONLY at that tag: it was taken out of the engine before the merge, as a method the sweep showed to be the wrong tool; opt-in per rotor, spread_line's TipCorrectionState, relax 0.1, eps_opt 0.25 chord floored at the element spacing)
date: 2026-11-13
path: CPU reference, MSVC, release (eps_LES = 2 dx; the trailed vorticity as Helmholtz-conservative interface jumps)
config: as the width record above, default width (2 dx), SPADE_V2_TIP=1
measure: line power, R/dx 19 = 17.485 W (off: 17.183 W); thrust +0.2% (off +1.2%); FM 0.784
reference: BEMT with Prandtl tip loss 19.771 W, the line -11.6% (off -13.1%)
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: line power, R/dx 25 = 17.108 W (off: 16.910 W); thrust +1.8% (off +1.6%); FM 0.821
reference: BEMT with Prandtl tip loss 19.771 W, the line -13.5% (off -14.5%)
tolerance: within 15% (V2, Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
```

## 2026-11-16: the gap follows the kernel's physical width, and the tip correction does not explain it (L72a, L72c)
Physics. With the kernel width held fixed in millimetres the power gap to the BEMT is converged in resolution to 0.3-0.5 points (6.1 mm: -14.8% at R/dx 19, -14.5% at 25; 8.0 mm: -13.1%, -12.6%); the apparent growth with
resolution in the two V2 blocks above was the kernel narrowing (eps = 2 dx: 8.0 mm at R/dx 19, 6.1 mm at 25). The gap shrinks with width with diminishing return (-14.8, -13.1, -12.2, -11.7% at 6.1, 8.0, 10.0, 12.0 mm,
R/dx 19) and does not close to the BEMT inside the kernel's six-cell support; the tip element's axial inflow at r/R 0.975 rises 0.29, 0.69, 1.18, 1.62 m/s against the BEMT's 6.5. n_sub 16 changes nothing (17.187 W against
17.183 W). The tip correction gains 1.5 points of power at R/dx 19 and 1.0 at 25 (it shrinks with eps_LES, which shrinks with the grid) and cannot remove the trend; its du is positive where the loading peaks (+2 m/s at r/R 0.875-0.925) and
negative at the root and the outermost element (-0.1 to -1.2 m/s) in every discretisation tried, a property of the loading and not of the scheme. It was removed from the engine before the merge (YAGNI; the code is at the tag physics/l72-tip-correction-334d28ef, with the design note tip-correction-design.md). The width is therefore a model parameter, to be chosen against
measured data (UIUC's static point) and not against the BEMT, which is itself an estimate: the rule is fixed in `.superpowers/sdd/2026-11-10-wake/line-width-choice-rule.md` before any measured-data run.
