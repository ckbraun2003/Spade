# S1, a sphere in a uniform stream -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; bands in `docs/design/test-docs/2026-10-13-solids-tolerances.md`). P2's checks: free slip at R/dx 6, the first step's solve and the closed faces. The drag and pressure comparisons enter with P3.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P2 checks ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the solve's final relative residual = 3.509e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the aperture divergence left, over the right side's = 3.274e-06
reference: 0 (a solved projection leaves rounding only)
tolerance: <= 2e-4 (twice the residual's band: the update's rounding; P2 plan, Task 7, fixed before the first run)
verdict: within
```

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P3 checks ===
commit: f4ddf75
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 0.01 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4915
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.24 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0813
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.32 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0612
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0390
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 6.70e-07
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 3.509e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
```

```text
=== validation record: S1 sphere in a uniform stream, R/dx 12, P3 checks ===
commit: f4ddf75
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 96^3 at dx 0.005 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4957
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.12 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0449
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.2 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0423
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0363
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 3.21e-05
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 1.189e-05
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the order check, e(12) / e(6) for C_p at r = 7R/6 = 0.461 (e(6) 0.0813, e(12) 0.0375)
reference: first order or better
tolerance: <= 0.7 (test-docs/2026-10-13-solids-tolerances.md, order checks)
verdict: within
```

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S1 sphere in a uniform stream, R/dx 6, P2 checks", master c7b3c06 -> P2c, every verdict unchanged: the solve's final relative residual: 3.509e-06 -> 2.807e-06; the aperture divergence left, over the right side's: 3.274e-06 -> 2.989e-06.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P2 checks ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the solve's final relative residual = 2.807e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the aperture divergence left, over the right side's = 2.989e-06
reference: 0 (a solved projection leaves rounding only)
tolerance: <= 2e-4 (twice the residual's band: the update's rounding; P2 plan, Task 7, fixed before the first run)
verdict: within
```

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S1 sphere in a uniform stream, R/dx 6, P3 checks", master c7b3c06 -> P2c, every verdict unchanged: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3: 6.70e-07 -> 5.20e-06; the worse of steps 1 and 3's final relative residuals: 3.509e-06 -> 2.807e-06.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P3 checks ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 0.01 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4915
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.24 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0813
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.32 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0612
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0390
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 5.20e-06
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 2.807e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
```

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S1 sphere in a uniform stream, R/dx 12, P3 checks", master c7b3c06 -> P2c, every verdict unchanged: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3: 3.21e-05 -> 2.75e-06; the worse of steps 1 and 3's final relative residuals: 1.189e-05 -> 8.046e-06.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 12, P3 checks ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 96^3 at dx 0.005 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4957
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.12 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0449
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.2 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0423
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0363
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 2.75e-06
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 8.046e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the order check, e(12) / e(6) for C_p at r = 7R/6 = 0.461 (e(6) 0.0813, e(12) 0.0375)
reference: first order or better
tolerance: <= 0.7 (test-docs/2026-10-13-solids-tolerances.md, order checks)
verdict: within
```

B5.1, 2026-10-26 (the two-level reduction order: blocks of 256 tiles on both twins, 7bdbc8c; Cameron's decision, backlog 45d11fe). "S1 sphere in a uniform stream, R/dx 6, P2 checks", 20c3bb1 -> B5.1, every verdict unchanged: the solve's final relative residual: 2.807e-06 -> 3.925e-06; the aperture divergence left, over the right side's: 2.989e-06 -> 3.392e-06.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P2 checks ===
commit: 7bdbc8c
date: 2026-10-26
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the solve's final relative residual = 3.925e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the aperture divergence left, over the right side's = 3.392e-06
reference: 0 (a solved projection leaves rounding only)
tolerance: <= 2e-4 (twice the residual's band: the update's rounding; P2 plan, Task 7, fixed before the first run)
verdict: within
```

B5.1, 2026-10-26 (the two-level reduction order: blocks of 256 tiles on both twins, 7bdbc8c; Cameron's decision, backlog 45d11fe). "S1 sphere in a uniform stream, R/dx 6, P3 checks", 20c3bb1 -> B5.1, every verdict unchanged: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3: 5.20e-06 -> 2.51e-06; the worse of steps 1 and 3's final relative residuals: 2.807e-06 -> 3.925e-06.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 6, P3 checks ===
commit: 7bdbc8c
date: 2026-10-26
path: CPU reference, MSVC 1943, release
config: 48^3 at dx 0.01 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4915
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.24 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0813
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.32 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0612
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0390
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 2.51e-06
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 3.925e-06
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
```

B5.1, 2026-10-26 (the two-level reduction order: blocks of 256 tiles on both twins, 7bdbc8c; Cameron's decision, backlog 45d11fe). "S1 sphere in a uniform stream, R/dx 12, P3 checks", 20c3bb1 -> B5.1, every verdict unchanged: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3: 4.96e-07 -> 2.46e-07 (P2c's block above says 2.75e-06; the run at 20c3bb1 printed 4.96e-07 before B5.1: a move after 824c492 with no record block, probably P2d (7df4ec9/b480559), unmeasured); the worse of steps 1 and 3's final relative residuals: 8.046e-06 -> 1.054e-05.

```text
=== validation record: S1 sphere in a uniform stream, R/dx 12, P3 checks ===
commit: 7bdbc8c
date: 2026-10-26
path: CPU reference, MSVC 1943, release
config: 96^3 at dx 0.005 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, U h/dx 0.1 (the drag also at 0.05), n_v 16 CG iterations from a cold start; solids: 1 parts, 1 walls, 0 sheets
measure: the added mass C_M = F_x h / (rho V U) from wall_load(), step 1 = 0.4957
reference: 0.5 (potential flow, an impulsively started sphere; Batchelor 1967, ch. 6)
tolerance: 0.5 +/- 0.12 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: max |C_p - C_p,pot| over 52 samples one cell out along the normal, step 3 = 0.0449
reference: potential flow's C_p at the same points, 1 - |u|^2/U^2 (Batchelor 1967, ch. 6)
tolerance: <= 0.2 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.1 = 0.0423
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2; a floor c0 of about 0.02 at R/dx 6 and 0.03 at 12 has no root cause yet, P3c)
verdict: report only
measure: the drag |F_x| / (1/2 rho U^2 pi R^2) from wall_load(), step 3, U h/dx 0.05 = 0.0363
reference: 0 (d'Alembert)
tolerance: none (report only: the lead's ruling 2026-10-22, d2d03ed, TD-2)
verdict: report only
measure: the side forces max(|F_y|, |F_z|) / (1/2 rho U^2 pi R^2), step 3 = 2.46e-07
reference: 0 (axisymmetric)
tolerance: <= 0.01 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the worse of steps 1 and 3's final relative residuals = 1.054e-05
reference: a converged solve
tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)
verdict: within
measure: the order check, e(12) / e(6) for C_p at r = 7R/6 = 0.461 (e(6) 0.0813, e(12) 0.0375)
reference: first order or better
tolerance: <= 0.7 (test-docs/2026-10-13-solids-tolerances.md, order checks)
verdict: within
```

## 2026-11-03: S1 on the device (B8)

GpuFluid, the whole step on the device of record (RTX 3060 Ti, driver 572.83), against the CPU's run of the same box over 32 steps: TD-14 as amended, bit-equal. The step time is report only (the lead, 2026-11-03). R/dx 6 and 12, the box as run_s1() builds it. The config line's doubled "solids:" is as printed.

```text
=== validation record: B8, the whole step on the device, S1 sphere in a uniform stream, R/dx 6, 48^3, GpuFluid (G3) against the CPU ===
commit: dfefd3fc (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8-records-dfefd3fc)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S1 as run_s1() sets it up (tests/test_airflow_solids_step.cpp, P3's checks): 48^3 at dx 0.01 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, u = U at the start, U h/dx 0.1 (h 0.001 s), CG floor 16, cap 32, the interior extrapolated; solids: solids: 1 parts, 1 walls, 0 sheets; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 14377312
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 13.2 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 1.804 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 1.804 m/s against 4 m/s; its mean speed at step 32 = 0.339727 m/s, the CPU's 0.339727 m/s (within 10% + 1e-04 m/s)`

```text
=== validation record: B8, the whole step on the device, S1 sphere in a uniform stream, R/dx 12, 96^3, GpuFluid (G3) against the CPU ===
commit: dfefd3fc (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8-records-dfefd3fc)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S1 as run_s1() sets it up (tests/test_airflow_solids_step.cpp, P3's checks): 96^3 at dx 0.005 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, u = U at the start, U h/dx 0.1 (h 0.0005 s), CG floor 16, cap 32, the interior extrapolated; solids: solids: 1 parts, 1 walls, 0 sheets; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 114131296
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 28.0 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 2.016 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 2.016 m/s against 4 m/s; its mean speed at step 32 = 0.339178 m/s, the CPU's 0.339178 m/s (within 10% + 1e-04 m/s)`

## 2026-11-03: S1 on the device, at the final commit (B8)

The same runs at 7230ec46, rebuilt on MSVC after the config line's fix: it now prints "solids:" once. Bit-equal again over 32 steps on the device of record (RTX 3060 Ti, driver 572.83); the step time is report only. Same identities as the blocks above, so these later blocks are the ones read.

```text
=== validation record: B8, the whole step on the device, S1 sphere in a uniform stream, R/dx 6, 48^3, GpuFluid (G3) against the CPU ===
commit: 7230ec46 (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8f-records-7230ec46)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S1 as run_s1() sets it up (tests/test_airflow_solids_step.cpp, P3's checks): 48^3 at dx 0.01 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, u = U at the start, U h/dx 0.1 (h 0.001 s), CG floor 16, cap 32, the interior extrapolated; solids: 1 parts, 1 walls, 0 sheets; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 14377312
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 13.5 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 1.804 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 1.804 m/s against 4 m/s; its mean speed at step 32 = 0.339727 m/s, the CPU's 0.339727 m/s (within 10% + 1e-04 m/s)`

```text
=== validation record: B8, the whole step on the device, S1 sphere in a uniform stream, R/dx 12, 96^3, GpuFluid (G3) against the CPU ===
commit: 7230ec46 (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8f-records-7230ec46)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S1 as run_s1() sets it up (tests/test_airflow_solids_step.cpp, P3's checks): 96^3 at dx 0.005 m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, other faces open, u = U at the start, U h/dx 0.1 (h 0.0005 s), CG floor 16, cap 32, the interior extrapolated; solids: 1 parts, 1 walls, 0 sheets; steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 114131296
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 29.5 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 2.016 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 2.016 m/s against 4 m/s; its mean speed at step 32 = 0.339178 m/s, the CPU's 0.339178 m/s (within 10% + 1e-04 m/s)`
