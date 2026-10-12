# C1, the lid-driven cavity -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). Reference data: `../ghia1982_cavity.hpp`. The blocks are the tests' printed output; MSVC wrote the section sign as code page 1252, converted here to UTF-8.

```text
=== validation record: C1 lid-driven cavity, Re 100, 32x32 ===
commit: 36153e0 (physics/airflow-core, rebased from dd497a4 with the same content; the solver of Tasks 1-9)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 32x32 (a quasi-2D slab, z periodic), Re 100, T 20 (1280 steps of h = 0.015625), n_v 2, visc_sweeps 8, rho 1, U 1
measure: max |u - u_Ghia| on x = 0.5 = 0.0064
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.06 (plan A Task 11, spec Â§11.2, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.0041
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.06 (plan A Task 11, spec Â§11.2, fixed before the first run)
verdict: within
```

```text
=== validation record: C1 lid-driven cavity, Re 100, 128x128 ===
commit: 36153e0 (physics/airflow-core, rebased from dd497a4 with the same content; the solver of Tasks 1-9)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 128x128 (a quasi-2D slab, z periodic), Re 100, T 40 (10240 steps of h = 0.003906), n_v 2, visc_sweeps 8, rho 1, U 1
measure: max |u - u_Ghia| on x = 0.5 = 0.0037
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (plan A Task 11, spec Â§11.2, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.0087
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (plan A Task 11, spec Â§11.2, fixed before the first run)
verdict: within
```
