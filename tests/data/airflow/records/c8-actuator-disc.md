# C8, the actuator disc's far wake -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). The reference is momentum theory's fully developed wake on the axis at z = 2R (README.md, C8).

```text
=== validation record: C8 actuator-disc far wake ===
commit: 05099fa (physics/airflow-core; the solver bit-identical to Tasks 1-9's, per that commit)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 384 steps
measure: axial velocity on the axis 2R below the disc, mean of the last 128 steps = 7.4124 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8)
tolerance: within 10% (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
```

## 2026-10-22: the verdict above is rounding-sensitive

Test/Docs, from Rendering's B4 run. This is prose, not a record: the readers, and the Validation readout (I2), read only the fenced blocks, so the readout still shows the block above as within until the ensemble's record is appended below.

- **The run:** rendering/airflow-gpu-b4-actuator `e965df2`, `TEST(GpuAirflowActuator, C8GpuGatherInTheCpuStep)`, 2026-10-21, MSVC release on the RTX 3060 Ti box. It runs the configuration above, unchanged, on the CPU step.
  - The control: the CPU's own increments, copied in each step, reproduce the CPU run to the bit.
  - The nudges: 110 of the disc's increments put 1 ulp up, the same every step, picked from seeds 0xC8, 0xC9 and 0xCA. 110 is how many the GPU gather differs on.
- **Where the runs part:** the probe 2R below agrees to 1e-5 on every run through step 128 (0.16 s). The nudged runs leave the CPU's by 1e-3 at steps 160, 146 and 154, and the GPU gather's at 169. The far wake is chaotic past about step 140 (`T_p`, about 0.18 s). C8's window, the last 128 steps (257 to 384), lies wholly past it.
- **C8's measure, the mean of the last 128 steps:**
  - The CPU: 7.412388 m/s, the block above, −2.2% against momentum theory's 7.5777.
  - The nudges 0xC8, 0xC9 and 0xCA: 7.479177, 7.058580 and 6.613182 m/s. That is 1.009, 0.952 and 0.892 of the CPU's, or −1.3%, −6.9% and −12.7% against momentum theory.
  - The GPU gather: 6.927015 m/s, 0.9345 of the CPU's (−8.6%).
- **So:** the block above reads its one run correctly, but C8's "within" rests on that one trajectory. A 1-ulp change in the forcing moves the same measure across the band, and seed 0xCA misses C8's 10%. Until a measure is agreed, C8's verdict is rounding-sensitive, neither within nor a miss.
- **Next:**
  - Test/Docs runs an ensemble of 1-ulp-nudged runs, CPU only, as queue jobs.
  - Physics and Test/Docs agree a measure whose verdict holds across that spread. Its band is fixed before the run that judges it (`TD-2`). The 10% stays argued from the literature, never from our own spread (01-verification, "Validation against published data").
  - C8 is re-cut only then, and its record appended.

```text
=== validation record: C8 actuator-disc far wake ===
commit: abaafcc (test-docs/c8-spread)
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 768 steps; eight runs that differ only by a nudge in the test, 1 ulp up on 110 increments, seed s: run 0 as it is, runs 1 to 7 seeds 0x1C8, 0x1C9, 0x1CA, 0x1CB, 0x1CC, 0x1CD, 0x1CE. So the spread is the measure's sensitivity to rounding, not a physical uncertainty
measure: the eight runs' mean of the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps 385 to 768 = 8.0333 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8); the vortex-cylinder model's mean over the same faces is 7.5909 m/s
tolerance: within 10% (plan A Task 11, spec §11.2, fixed before the first run; the measure and the eight runs fixed before this run, docs/design/test-docs/2026-10-22-c8-spread.md)
verdict: within
measure: the worst of the eight runs, run 6, the same mean = 8.0597 m/s
reference: momentum theory, 7.5777 m/s, as above
tolerance: within 10% (as above: the verdict holds across the spread)
verdict: within
measure: the eight runs' largest deviation from their mean, the same mean = 0.0437 m/s
reference: the spread's size: past T_p the GPU's band is 4x it (test-docs/01-verification.md, a case judged past its horizon)
tolerance: none
verdict: report only
measure: C8's first measure, the axis 2R below over steps 257 to 384, run 0 (the eight runs 6.3611 to 7.5019) = 7.4124 m/s
reference: momentum theory, 7.5777 m/s, as above
tolerance: none (report only since 2026-10-22: the axis sits in the disc's unloaded hub)
verdict: report only
```

Physics' note on this record (2026-10-22): The disc mean reads about 6% above momentum theory's uniform disc. spread_actuator's 48 points load the disc unevenly: an unloaded hub (inner ring at 0.408R, Peskin's reach 2 cells), and 1.34× the uniform thrust per area inside 0.5R. Annular momentum theory gives up to √1.34 ≈ 1.16× there, and mixing reduces it. The bias belongs to the disc's point loading, not the solver; it is within the 10% band.

The measure and the eight runs were fixed, and signed off by Physics, before this run: `docs/design/test-docs/2026-10-22-c8-spread.md`. Its 16-run measurement predicted an ensemble mean of +5.6% to +6.2% and a worst run of +6.1% to +6.6% against momentum theory; this run gives +6.0% and +6.4%. The first block above stays, read as the one run it was.

## 2026-10-22: B4, the GPU gather in the CPU step

```text
=== validation record: C8 actuator-disc far wake, GPU gather in the CPU step ===
commit: b8ec4c9 (rendering/c8-record-fix; queue ticket spade-rendering-b4c8-c8fix-b8ec4c9)
date: 2026-10-22
path: the CPU step (release) with airflow_actuator_gather's increments (plan B, B4), on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no)
config: as C8: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 768 steps; the gather's increments, 110 of 1593344 differing from spread_actuator's by 2.38e-07 m/s at most, copied in each step
measure: the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps 385 to 768 = 8.0068 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8)
tolerance: within 10% (C8's: plan A Task 11, spec §11.2)
verdict: within
measure: the same mean = 8.0068 m/s
reference: C8's one-ulp ensemble, the mean of its eight runs = 8.0333 m/s (C8's record, abaafcc)
tolerance: within 4x the ensemble's largest deviation, 4 x 0.0437 = 0.1748 m/s (test-docs/01-verification.md, a case judged past its horizon)
verdict: within
measure: the 0.5R mean's largest |gather - CPU| through step 104 (T_p) = 7.21e-06 m/s
reference: the CPU's run, spread_actuator() each step
tolerance: within 4x the measured gap, never under 4 ulps, 2.88e-05 m/s (TD-14; the gap measured 2026-10-22 at 3eacd6a)
verdict: within
measure: the axis probe's largest |gather - CPU| through step 104 (T_p) = 3.45e-05 m/s
reference: the CPU's run, spread_actuator() each step
tolerance: within 4x the measured gap, never under 4 ulps, 1.38e-04 m/s (TD-14; the gap measured 2026-10-22 at 3eacd6a)
verdict: within
measure: the step at which the run leaves the CPU's by 1e-3 on the axis 2R below = 169
reference: the one-ulp ensemble's earliest member, step 141 of 16 (2026-10-22-c8-spread.md)
tolerance: no sooner than step 141 (2026-10-22-c8-spread.md, fixed before the run)
verdict: within
measure: the CPU's run, the same 0.5R mean = 8.0397 m/s
reference: none
tolerance: none (report only)
verdict: report only
```

## 2026-10-26: K2, the exact gather in the CPU step, bit-equal

```text
=== validation record: C8 actuator-disc far wake, GPU gather in the CPU step ===
commit: a0043c9 (rendering/k2b-record-fix; merged at e57f16a)
date: 2026-10-26
path: the CPU step (release) with airflow_actuator_gather's increments (plan B, B4; its square roots fp32_math's sqrt32_ftz), on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no)
config: as C8: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 768 steps; the gather's increments copied in each step
measure: the gathered increments differing from spread_actuator()'s in any bit = 0 of 1593344
reference: spread_actuator(), the CPU step's own increments
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: an exact kernel on the device of record)
verdict: within
measure: both probes' values (the axis and the 0.5R mean, 2R below) differing from the CPU's run over steps 1 to 768 = 0 of 1536
reference: the CPU's run, spread_actuator() each step
tolerance: 0, bit-equal: the run is the CPU's run (TD-14 as amended 2026-10-22: an exact kernel on the device of record)
verdict: within
measure: the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps 385 to 768 = 8.0397 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8)
tolerance: within 10% (C8's: plan A Task 11, spec §11.2; run 0's verdict, the run being C8's run 0: past T_p C8's own verdict is its ensemble's, abaafcc)
verdict: within
```

Test/Docs' note on this block (2026-10-26): the run is C8's run 0 to the bit. Besides the 0 of 1536, its 0.5R mean, 8.0397 m/s, is run 0's 8.039714 (`docs/design/test-docs/2026-10-22-c8-spread.md`) and the CPU's run in B4's block above. The path line's "flush-to-zero supported: no" is the device's report of a mode a kernel could request. With none requested, its arithmetic flushes (a sign-preserving FTZ and DAZ), and the CPU step flushes as it does (`1130608`): that is `TD-14`'s "both twins flush".

## 2026-10-27: the disc's density follows R/dx, 5 rings of 32

```text
=== validation record: C8 actuator-disc far wake ===
commit: 80902bc (physics/airflow-dense-disc, the density rule's work in progress with master 01ba8a9 merged; squashed into the disc's commit before the merge; queue ticket spade-physics-dense-cpu2-80902bc)
date: 2026-10-27
path: CPU reference, MSVC 1943, release
config: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 768 steps; eight runs that differ only by a nudge in the test, 1 ulp up on 110 increments, seed s: run 0 as it is, runs 1 to 7 seeds 0x1C8, 0x1C9, 0x1CA, 0x1CB, 0x1CC, 0x1CD, 0x1CE. So the spread is the measure's sensitivity to rounding, not a physical uncertainty
measure: the eight runs' mean of the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps 385 to 768 = 7.8893 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8); the vortex-cylinder model's mean over the same faces is 7.5909 m/s
tolerance: within 10% (plan A Task 11, spec §11.2, fixed before the first run; the measure and the eight runs fixed before this run, docs/design/test-docs/2026-10-22-c8-spread.md)
verdict: within
measure: the worst of the eight runs, run 6, the same mean = 7.9796 m/s
reference: momentum theory, 7.5777 m/s, as above
tolerance: within 10% (as above: the verdict holds across the spread)
verdict: within
measure: the eight runs' largest deviation from their mean, the same mean = 0.0902 m/s
reference: the spread's size: past T_p the GPU's band is 4x it (test-docs/01-verification.md, a case judged past its horizon)
tolerance: none
verdict: report only
measure: C8's first measure, the axis 2R below over steps 257 to 384, run 0 (the eight runs 7.7495 to 8.0064) = 7.9698 m/s
reference: momentum theory, 7.5777 m/s, as above
tolerance: none (report only since 2026-10-22: the axis sits in the disc's unloaded hub)
verdict: report only
```

```text
=== validation record: C8 actuator-disc far wake, GPU gather in the CPU step ===
commit: 80902bc (physics/airflow-dense-disc, as the block above; queue ticket spade-physics-dense-gpu2-80902bc)
date: 2026-10-27
path: the CPU step (release) with airflow_actuator_gather's increments (plan B, B4; its square roots fp32_math's sqrt32_ftz), on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no)
config: as C8: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T 1.2315 N, v_h 4 m/s, still air, inviscid, h = 0.001250, n_v 2, 768 steps; the gather's increments copied in each step
measure: the gathered increments differing from spread_actuator()'s in any bit = 0 of 1593344
reference: spread_actuator(), the CPU step's own increments
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: an exact kernel on the device of record)
verdict: within
measure: both probes' values (the axis and the 0.5R mean, 2R below) differing from the CPU's run over steps 1 to 768 = 0 of 1536
reference: the CPU's run, spread_actuator() each step
tolerance: 0, bit-equal: the run is the CPU's run (TD-14 as amended 2026-10-22: an exact kernel on the device of record)
verdict: within
measure: the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps 385 to 768 = 7.9120 m/s
reference: momentum theory, v_h (1 + 2/sqrt 5) = 7.5777 m/s (tests/data/airflow/README.md, C8)
tolerance: within 10% (C8's: plan A Task 11, spec §11.2; run 0's verdict, the run being C8's run 0: past T_p C8's own verdict is its ensemble's, the 5 x 32 disc's of 2026-10-27)
verdict: within
```

Physics' note on these blocks (2026-10-27): the library's actuator disc takes its point density from R/dx (the lead's ruling of 2026-10-27: rings clamp(ceil(R/2dx), 3, 10), angles the least of 16, 32 and 64 at or above pi R/dx, so the points stay about 2 dx apart or closer; D1 at R/dx 19 keeps the 10 rings of 64 that ended triage 2's 85-150 m/s jet, and the editor's Run air at R/dx 3 keeps the 3 x 16). C8's disc at R/dx 10 is 5 rings of 32 (160 points). Against the 3 x 16 ensemble above, its mean goes from 8.0333 to 7.8893 m/s (+6.0% to +4.1% against momentum theory), the worst run from 8.0597 to 7.9796 (run 6 both times), the largest deviation from 0.0437 to 0.0902 m/s, and the axis measure from 7.4124 to 7.9698 m/s (the eight runs 6.3611-7.5019, now 7.7495-8.0064). The 5 x 32's inner ring is at 0.316R (3.16 cells), so the four axis faces stay nearly unloaded, at 0.001x the uniform disc's thrust per area (the 3 x 16 at 0): the CPU block's "unloaded hub" holds. Inside 0.5R its points put 0.80x the uniform thrust (the 3 x 16 1.34x), spread to 0.99x over the measure's 80 faces, so the remaining +4.1% is not the 0.5R loading. The GPU block's run is the CPU's run 0 to the bit (0 of 1536), and its verdict is run 0's: this ensemble's verdict is the CPU block's. GpuAirflowActuator's band off the device of record is now 7.8893 m/s within 4 x 0.0902.
