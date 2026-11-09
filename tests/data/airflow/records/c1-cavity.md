# C1, the lid-driven cavity -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). Reference data: `../ghia1982_cavity.hpp`. The blocks are the tests' printed output, UTF-8 as printed. (Correction before review, 2026-10-12: the first two blocks' section signs had been double-encoded when pasted, decoded as code page 1252 by mistake; restored to the printed bytes. No value changed.)

```text
=== validation record: C1 lid-driven cavity, Re 100, 32x32 ===
commit: 36153e0 (physics/airflow-core, rebased from dd497a4 with the same content; the solver of Tasks 1-9)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 32x32 (a quasi-2D slab, z periodic), Re 100, T 20 (1280 steps of h = 0.015625), n_v 2, visc_sweeps 8, rho 1, U 1
measure: max |u - u_Ghia| on x = 0.5 = 0.0064
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.06 (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.0041
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.06 (plan A Task 11, spec §11.2, fixed before the first run)
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
tolerance: <= 0.02 (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.0087
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
```

```text
=== validation record: C1 lid-driven cavity, Re 400, 128x128 ===
commit: 81a87da (physics/airflow-core)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 128x128 (a quasi-2D slab, z periodic), Re 400, T 60 (15360 steps of h = 0.003906), n_v 2, visc_sweeps 8, rho 1, U 1
measure: max |u - u_Ghia| on x = 0.5 = 0.0076
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a suspected typo in the original) = 0.0065
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (plan A Task 11, spec §11.2, fixed before the first run)
verdict: within
```

```text
=== validation record: C1 lid-driven cavity, Re 100, 128x128, u_lat 0.1, LBM, gradual underflow (the flush scope off) ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.128 (tau 0.8840), Re 100, T 50 L/U (64000 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; step unscoped, gradual underflow (SPADE_LBM_GRADUAL set), 6 thread(s); final-f digest 0x7a0b230dd924d1fb; the scoped run of the same configuration: digest 0x5b1f30079bd5d15e; the first single-thread probe of 2026-11-05 (5d385e4) printed the same measures
measure: max |u - u_Ghia| on x = 0.5 = 0.00505
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.00704
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.00000
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.00297
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.803 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C1 lid-driven cavity, Re 100, 128x128, u_lat 0.1, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.128 (tau 0.8840), Re 100, T 50 L/U (64000 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; step under the flush scope (FTZ|DAZ) (SPADE_LBM_GRADUAL unset), 6 thread(s); final-f digest 0x5b1f30079bd5d15e; the gradual-underflow run of the same configuration: digest 0x7a0b230dd924d1fb; the flush moved bits (subnormals in the decayed corner fields after 64000 steps of a walled cavity) and no printed measure
measure: max |u - u_Ghia| on x = 0.5 = 0.00505
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.00704
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.00000
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.00297
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.727 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C1 lid-driven cavity, Re 400, 128x128, u_lat 0.1, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.032 (tau 0.5960), Re 400, T 190 L/U (243200 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; step under the flush scope (FTZ|DAZ) (SPADE_LBM_GRADUAL unset), 6 thread(s); final-f digest 0xe8ba20d1e44ad64e
measure: max |u - u_Ghia| on x = 0.5 = 0.00458
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a suspected typo in the original) = 0.00367
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.00000
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.01622
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.923 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C1 lid-driven cavity, Re 100, 128x128, u_lat 0.05, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at c38c4878 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.05 (Ma 0.087), nu_lat 0.064 (tau 0.6920), Re 100, T 50 L/U (128000 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; step under the flush scope (FTZ|DAZ) (SPADE_LBM_GRADUAL unset), 6 thread(s); final-f digest 0x00f76c7967fa7059
measure: max |u - u_Ghia| on x = 0.5 = 0.00513
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5 = 0.00756
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.00000
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.00332
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.289 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C1 lid-driven cavity, Re 400, 128x128, u_lat 0.05, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at c38c4878 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.05 (Ma 0.087), nu_lat 0.016 (tau 0.5480), Re 400, T 190 L/U (486400 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; step under the flush scope (FTZ|DAZ) (SPADE_LBM_GRADUAL unset), 6 thread(s); final-f digest 0x0030fc995c04013e
measure: max |u - u_Ghia| on x = 0.5 = 0.00392
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a suspected typo in the original) = 0.00377
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.00000
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.01874
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.661 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

The same on the device, one run (tests/test_gpu_lbm.cpp, GpuLbmCase.OneCaseInLockStepWithTheCpu): the lid-driven cavity at Re 100 stepped on the device and on the CPU together, the populations compared word for word every 64 steps. The first run at 20d98cc8 printed the same digest in 268.9 s; the rerun at d39efd2f took 474.8 s (shared box).

```text
=== validation record: C1 lid-driven cavity, Re 100, 128x128, u_lat 0.1, LBM, on the device in lock step with the CPU ===
commit: physics/lbm-gpu-t5 (T5; run at d39efd2f, SPADE_LBM_VALIDATION=1 SPADE_LBM_GPU_CASE=c1:100:0.1)
date: 2026-11-09
path: the device of record (the chain of lbm_macro and lbm_collide_stream, the staged lbm_f/lbm_u buffers) in lock step with the CPU reference, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes; CPU MSVC 1943, release, 6 threads (WorkerPool)
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.128 (tau 0.8840), Re 100, T 50 L/U (64000 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32; the CPU step under the flush scope, the populations compared every 64 steps (1000 samples); final-f digest 0x5b1f30079bd5d15e (the scoped CPU run's digest above, equal)
measure: words of f differing from the CPU at the 1000 sample steps = 0
reference: the CPU reference's populations, word for word (TD-14: bit equality is for numbers, not NaN payloads, so the CPU field is checked finite at each sample)
tolerance: exact (TD-14)
verdict: within
```
