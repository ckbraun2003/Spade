# C8q, the reference quad's downwash -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; the case and its bands in `docs/design/test-docs/2026-10-19-end-goal-checks.md` §5). The library's reference quad, its parts cleared, held at hover in still air at R/dx 10: each rotor's mean downwash over a 0.5R disc 2R below its hub against momentum theory's far wake (README.md, C8q), on four runs a rounding nudge apart.

```text
=== validation record: C8q reference quad ===
commit: cff9429 (test-docs/c8q)
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: the library's reference quad (assets/library/drones/reference_quad.model.yaml), its parts cleared as the editor's airflow chapter flies it, held level at the box's centre in still air, no floor; actuator disc · 128³, dx 7.61719 mm, R/dx 10; a 0.975 m box, dx 7.62 mm, R/dx 10.00, h = dx/8 = 0.000952 s, air's kinematic viscosity 1.461e-05 m2/s (C8 ran inviscid: the difference was ruled immaterial before the run, Physics 2026-10-22: viscous spreading about 0.05R by 0.5 s, the probe disc's edge about 0.23R inside the slipstream); armed at the hover throttle 0.1655 from t = 0; the window [0.3, 0.5] s, 6 frames a run. Four runs that differ only by a nudge in the test, 1 ulp up on 110 of the discs' increments, seed s: run 0 as it is, runs 1 to 3 seeds 0x1C8, 0x1C9, 0x1CA. So the spread is the measure's sensitivity to rounding, not a physical uncertainty
measure: rotor 0's mean downwash over the 0.5R disc 2R below, the four runs' mean = 8.0699 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, the four runs' mean = 9.8807 m/s (tests/data/airflow/README.md, C8 and C8q)
tolerance: within 10% (C8's, plan A Task 11; C8q's bands and its four runs fixed before the first run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)
verdict: MISS
measure: rotor 1's mean downwash over the 0.5R disc 2R below, the four runs' mean = 8.0587 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, the four runs' mean = 9.8807 m/s (tests/data/airflow/README.md, C8 and C8q)
tolerance: within 10% (C8's, plan A Task 11; C8q's bands and its four runs fixed before the first run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)
verdict: MISS
measure: rotor 2's mean downwash over the 0.5R disc 2R below, the four runs' mean = 8.0635 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, the four runs' mean = 9.8807 m/s (tests/data/airflow/README.md, C8 and C8q)
tolerance: within 10% (C8's, plan A Task 11; C8q's bands and its four runs fixed before the first run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)
verdict: MISS
measure: rotor 3's mean downwash over the 0.5R disc 2R below, the four runs' mean = 8.0722 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, the four runs' mean = 9.8807 m/s (tests/data/airflow/README.md, C8 and C8q)
tolerance: within 10% (C8's, plan A Task 11; C8q's bands and its four runs fixed before the first run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)
verdict: MISS
measure: the four rotors' means, the four runs' mean, off their mean by at most = 0.0009
reference: the four rotors are equal statistically (quarter-turn symmetry; Physics, 2026-10-19)
tolerance: <= 0.1 (as above)
verdict: within
measure: the worst run by the rotor band, run 0, off momentum theory by at most = 0.1888
reference: momentum theory, as above
tolerance: <= 0.1 (as above: the verdict holds across the spread)
verdict: MISS
measure: the worst run by the mutual band, run 2, the four off their mean by at most = 0.0089
reference: the four rotors are equal statistically, as above
tolerance: <= 0.1 (as above: the verdict holds across the spread)
verdict: within
measure: the mutual spread, (max - min) / mean of the four rotors' means, the four runs' mean = 0.0017
reference: none (evidence for any later tightening, set in advance)
tolerance: none
verdict: report only
measure: rotor 0's spread across the four runs, (max - min) / mean = 0.0117
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: rotor 1's spread across the four runs, (max - min) / mean = 0.0082
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: rotor 2's spread across the four runs, (max - min) / mean = 0.0157
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: rotor 3's spread across the four runs, (max - min) / mean = 0.0154
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
```

## 2026-10-22: a finding, open with Physics

Test/Docs, on the lead's word. This is prose, not a record: the readers read only the fenced block above.

- **What missed:** every rotor reads 0.816 to 0.817 of momentum theory in the runs' mean, and the worst run 0.811. The deficit is the same on every rotor and in every run: the mutual band holds at 0.0009, and the spread across the runs is 0.8% to 1.6%. The batch smoke's airflow chapter, the same quad at 32³ and R/dx 3, reads the same 0.817 (the lead), so resolution doesn't move it.
- **Ruled out before any further run** (Test/Docs and Physics, reading the code):
  - The disc's force and the reference's thrust are the same `thrust_coeff ω²` from the same rotor row, with the same radius and density (`engine/vehicles/rotor_discs.cpp`, `sandbox/end_goal_checks.hpp`).
  - The 48-point spread sums to T exactly, and it is C8's own spread.
  - The box's faces are all open with no floor, as in C8.
- **Open:** the one difference left from C8 is that the quad's discs carry torque. P1's swirl puts `torque / (48 r)` on each point, and `torque_coeff / thrust_coeff` = 0.0102 m, so Q/(TR) ≈ 0.13. C8's disc had none. A test-only run with the discs' torque at 0 decides it (the lead's grant, on Physics' word). Physics expects it back near C8's +5.7%, which would make P1's swirl the cause and the fix Physics', in the engine.
- Whatever that run finds is appended below, dated. The block above is never edited.
