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
