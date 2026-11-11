# L61 V1, the grid-free BEMT against UIUC's measured 10x5 -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). The bands are Test/Docs',
`docs/design/test-docs/2026-11-11-l61-actuator-line-bands.md`, fixed before the first run. The measured lines are the block
`AirflowBemtUiuc.V1TheEstimatedPolarAgainstTheMeasured10x5` prints (tests/test_airflow_bemt_uiuc.cpp); the polar is the estimated one
of engine/physics/airflow/blade.hpp, written before any comparison and NOT calibrated: V1a's static point stood inside its trigger.

```text
=== validation record: L61 V1 BEMT on uiuc_apce_10x5, estimated polar ===
commit: 8f903f4 (physics/line, the V1 build; squashed with the same content into the P4a commit)
date: 2026-11-11
path: CPU reference, MSVC, release (tests/bemt_reference.hpp, Prandtl tip loss on)
config: uiuc_apce_10x5 stations (tests/library_props.gen.hpp), 2 blades, D 0.254 m, 5000 rpm, rho 1.225 kg/m3, 128 annuli, bisection 60; estimated polar alpha_0 -3 deg, slope 5.5/rad, attached to +11 and -17 deg then blended over 14 deg to the flat plate, C_d 0.03 + 0.02 C_l^2; not calibrated
measure: static C_T = 0.09554
reference: UIUC Propeller Data Site Vol. 1, apce_10x5 static point, C_T 0.09636 (tests/library_props.gen.hpp, uiuc_apce_10x5_ct row J 0)
tolerance: within 10% (V1a, a trigger for calibration; Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: static C_Q = 0.005864
reference: UIUC Propeller Data Site Vol. 1, apce_10x5 static point, C_Q 0.005870 (tests/library_props.gen.hpp, uiuc_apce_10x5_cq row J 0)
tolerance: within 15% (V1a, a trigger for calibration; Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_T - C_T,UIUC| at J 0.2, share of the static value = 0.079
reference: UIUC apce_10x5 run, C_T 0.08340 at J 0.2 (uiuc_apce_10x5_ct); the line's C_T 0.07579
tolerance: <= 0.16 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_Q - C_Q,UIUC| at J 0.2, share of the static value = 0.068
reference: UIUC apce_10x5 run, C_Q 0.006191 at J 0.2 (uiuc_apce_10x5_cq); the line's C_Q 0.005794
tolerance: <= 0.21 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_T - C_T,UIUC| at J 0.3, share of the static value = 0.031
reference: UIUC apce_10x5 run, C_T 0.06440 at J 0.3 (uiuc_apce_10x5_ct); the line's C_T 0.06138
tolerance: <= 0.19 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_Q - C_Q,UIUC| at J 0.3, share of the static value = 0.045
reference: UIUC apce_10x5 run, C_Q 0.005655 at J 0.3 (uiuc_apce_10x5_cq); the line's C_Q 0.005392
tolerance: <= 0.25 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_T - C_T,UIUC| at J 0.4, share of the static value = 0.002
reference: UIUC apce_10x5 run, C_T 0.04525 at J 0.4 (uiuc_apce_10x5_ct); the line's C_T 0.04540
tolerance: <= 0.22 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: |C_Q - C_Q,UIUC| at J 0.4, share of the static value = 0.008
reference: UIUC apce_10x5 run, C_Q 0.004640 at J 0.4 (uiuc_apce_10x5_cq); the line's C_Q 0.004685
tolerance: <= 0.30 (V1b, the polar stood: Test/Docs 2026-11-11 bands note, fixed before the first run)
verdict: within
measure: C_T at J 0.5 = 0.02795
reference: none (report only; UIUC C_T 0.02854 at J 0.5, Test/Docs: J 0.5 and the efficiency are report only)
tolerance: none (report only)
verdict: report only
measure: C_Q at J 0.5 = 0.003601
reference: none (report only; UIUC C_Q 0.003563 at J 0.5)
tolerance: none (report only)
verdict: report only
```

## 2026-11-11: what this record does not hold

Physics. The polar was fixed before the run (the design note section 1) and never touched; the static point landed at -0.9% (C_T) and -0.1% (C_Q).
Test/Docs' note calls a static pass a threshold, not a validation: the estimated polar's plausible spread is about +-14%, so a hit this close is
partly luck. The held-out J is the test, and the largest miss is 7.9% of static (C_T, J 0.2) against a band of 16%. The polar's reach at J 0.2 is
therefore low by 9%; that is the held-out error to remember before the grid-resolved runs (V2).
