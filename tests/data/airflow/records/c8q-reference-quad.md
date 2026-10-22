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

## 2026-10-22: the run without the discs' swirl

Test/Docs: Physics' discriminating run, one run (`EndGoalC8q.ReferenceQuadWithoutTheDiscsSwirl` at `f8fbbae`, ticket `spade-testdocs-c8q-notorque-f8fbbae`). It is run 0's case with every rotor's torque coefficient at 0. The hover throttle came out the same, 0.1655, and so did the reference, 9.8807 m/s, so only the discs' torque changed.
- **Each rotor:** 0.8567, 0.8463, 0.8505 and 0.8527 of momentum theory (u 8.36 to 8.46 m/s). The four are off their mean by 0.0061 at most.
- **Against run 0 with the swirl** (0.811 to 0.814), the swirl takes about 4%. The rest stays without it: about 15% below momentum theory, and about 19% below C8's +5.7%.
- So the swirl is not the cause of most of the miss. Physics' next step, as Physics set it beforehand: one rotor alone through `LiveRun`. The finding stays open with Physics.

## 2026-10-22: rotor 0 alone, without swirl

Test/Docs: Physics' second discriminating run, report only and not a verdict (`EndGoalC8q.RotorZeroAloneWithoutSwirl` at `3aac467`, ticket `spade-testdocs-c8q-rotor0-3aac467`; the lead's grant, on Physics' word).
- **The case:** run 0's, without swirl, with rotors 1 to 3 pushing nothing. Their thrust coefficient is the least normal float, about 1e-32 N, since a model refuses 0. The hover throttle was trimmed on the four rotors first and came out at 0.1655, so rotor 0's reference is run 0's, 9.8807 m/s.
- **Rotor 0:** 10.4908 m/s, 1.0617 of momentum theory. That is within a percent of C8's +5.7%, a lone uniform disc read through the same 0.5R probe.
- **So the four slipstreams together make the miss.** Alone, rotor 0 reads as C8's disc does. With its three neighbours, it reads 0.85 without swirl and 0.81 with it.
- **Physics' reading, set before the run:** per-rotor momentum theory is the wrong reference for a quad whose rotor tips are 0.9R apart. C8q's reference and band are then a spec change for the lead to decide (Physics and Test/Docs). The finding stays open until it is decided.

## 2026-10-22: the bands changed before their run, and the miss in corrected terms

Test/Docs, in Physics' words (Physics checked them, 2026-10-22). C8q's bands were changed after the MISS above, and before the new banded measure's first run (`docs/design/test-docs/2026-10-19-end-goal-checks.md` §5, `eeef10f` and `7bc53de`; Physics' proposal, the lead's approval). The MISS block above stays as it came out.
- **Banded now:** rotor 0 alone, in the quad's own case and with its own torque, against momentum theory in C8's [0.9, 1.1], on its four runs' mean and its worst run. The four-rotor quad's symmetry band stays, on the same ensemble rule.
- **Report only now:** the quad's per-rotor ratios. They have no closed-form reference, and none is set from our own result.
- **The prose above says more than is known.** "So the four slipstreams together make the miss" reads, in corrected terms: the deficit comes with the four rotors together. Linear mutual induction gives only about 3% of it. That treats each neighbour's inflow as a sink at its disc, with the neighbours at 2.89R and 4.08R and the probe 2R below. The rest is the wakes' mixing or the open box's response to four times the flux, and nothing yet separates the two.
- **The two diagnostics above were one run each,** report only, not ensembles: 0.846 to 0.857 without the discs' torque, and 1.0617 for rotor 0 alone without it.
- **The four-wake reference** is the measured-data case queued in Physics' demonstration plan (item 10, master `2f29a32`), after the GPU solver.
- The finding above is closed by the change. The new banded run's record is appended below when it has run.

```text
=== validation record: C8q reference quad ===
commit: cc38924 (master)
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: the library's reference quad (assets/library/drones/reference_quad.model.yaml), its parts cleared as the editor's airflow chapter flies it, held level at the box's centre in still air, no floor; a 0.975 m box, dx 7.62 mm, R/dx 10.00, h = dx/8 = 0.000952 s, air's kinematic viscosity 1.461e-05 m2/s (C8 ran inviscid: the difference was ruled immaterial before the run, Physics 2026-10-22: viscous spreading about 0.05R by 0.5 s, the probe disc's edge about 0.23R inside the slipstream); armed at the hover throttle 0.1655 from t = 0, trimmed on the four-rotor model; the window [0.3, 0.5] s, 6 frames a run. Two ensembles, as C8q's bands were changed on 2026-10-22 before this run (docs/design/test-docs/2026-10-19-end-goal-checks.md section 5): rotor 0 alone, actuator disc · 128³, dx 7.61719 mm, R/dx 10, with its own torque, rotors 1 to 3 pushing nothing (thrust coefficient the least normal float, torque 0); and the quad as flown, actuator disc · 128³, dx 7.61719 mm, R/dx 10. Each is four runs that differ only by a nudge in the test, 1 ulp up on 110 of the discs' increments, seed s: run 0 as it is, runs 1 to 3 seeds 0x1C8, 0x1C9, 0x1CA. So each spread is the measure's sensitivity to rounding, not a physical uncertainty
measure: rotor 0 alone's mean downwash over the 0.5R disc 2R below, its four runs' mean = 10.5364 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) from the rotor's thrust, its four runs' mean = 9.8807 m/s (tests/data/airflow/README.md, C8 and C8q)
tolerance: within 10% (C8's, plan A Task 11; C8q's bands as changed on 2026-10-22, fixed before this run, docs/design/test-docs/2026-10-19-end-goal-checks.md section 5)
verdict: within
measure: rotor 0 alone's worst run, run 2, off momentum theory by = 0.0666
reference: momentum theory, as above
tolerance: <= 0.1 (as above: the verdict holds across the spread)
verdict: within
measure: rotor 0 alone's spread across its four runs, (max - min) / mean = 0.0005
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: the quad's rotor 0, its downwash over momentum theory's, its four runs' mean = 0.8167
reference: none (the four rotors together: their wakes' interaction and the open box's response to four times the flux, not separated; no closed-form reference, Physics 2026-10-22)
tolerance: none
verdict: report only
measure: the quad's rotor 1, its downwash over momentum theory's, its four runs' mean = 0.8156
reference: none (the four rotors together: their wakes' interaction and the open box's response to four times the flux, not separated; no closed-form reference, Physics 2026-10-22)
tolerance: none
verdict: report only
measure: the quad's rotor 2, its downwash over momentum theory's, its four runs' mean = 0.8161
reference: none (the four rotors together: their wakes' interaction and the open box's response to four times the flux, not separated; no closed-form reference, Physics 2026-10-22)
tolerance: none
verdict: report only
measure: the quad's rotor 3, its downwash over momentum theory's, its four runs' mean = 0.8170
reference: none (the four rotors together: their wakes' interaction and the open box's response to four times the flux, not separated; no closed-form reference, Physics 2026-10-22)
tolerance: none
verdict: report only
measure: the quad's rotors' means, its four runs' mean, off their mean by at most = 0.0009
reference: the four rotors are equal statistically (quarter-turn symmetry; Physics, 2026-10-19)
tolerance: <= 0.1 (C8q's mutual band, fixed before the first run, as above)
verdict: within
measure: the quad's worst run by the mutual band, run 2, its rotors off their mean by at most = 0.0089
reference: the four rotors are equal statistically, as above
tolerance: <= 0.1 (as above: the verdict holds across the spread)
verdict: within
measure: the quad's mutual spread, (max - min) / mean of its rotors' means, its four runs' mean = 0.0017
reference: none (evidence for any later tightening, set in advance)
tolerance: none
verdict: report only
measure: the quad's rotor 0's spread across its four runs, (max - min) / mean = 0.0117
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: the quad's rotor 1's spread across its four runs, (max - min) / mean = 0.0082
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: the quad's rotor 2's spread across its four runs, (max - min) / mean = 0.0157
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
measure: the quad's rotor 3's spread across its four runs, (max - min) / mean = 0.0154
reference: none (the measure's sensitivity to rounding)
tolerance: none
verdict: report only
```
