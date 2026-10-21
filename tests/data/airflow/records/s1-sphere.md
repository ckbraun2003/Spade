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
