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
