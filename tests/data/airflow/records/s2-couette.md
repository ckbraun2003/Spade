# S2, circular Couette flow -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; bands in `docs/design/test-docs/2026-10-13-solids-tolerances.md`). S2 checks the solution, not the wall treatment: at R1/dx 16 it passes with the viscous wall links (0.0052) and also without them (0.0602, P2 Task 8's mutation). The planar Couette check is the one that tells the two treatments apart, with its negative control in the same block. Each S2 record carries its last step's pressure residual, which shows the run reached the step's fixed point at n_v 4. R1/dx 32 and the order check are opt-in (`SPADE_AIRFLOW_VALIDATION`). S2 skips in debug builds under TD-5's "debug: runs in release" terms (100 s in debug, 15 s in release).

```text
=== validation record: S2 circular Couette flow, R1/dx 16 ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: a 68x1x68 slab (y periodic), dx 1 mm, R1 16 mm turning at Omega R1 = 0.1 m/s inside a fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, visc_sweeps 8
measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = 0.0052
reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)
tolerance: <= 0.11 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: closed faces off their rigid velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the mean u_theta over the measured cells = 0.03719 m/s
reference: positive, the rotation's sense
tolerance: > 0 (the cylinder turns the air its own way)
verdict: within
measure: the last step's relative pressure residual = 1.715e-06
reference: a converged solve (the step's fixed point)
tolerance: <= 1e-04 (kResidualAnnounce)
verdict: within
```

```text
=== validation record: S2 circular Couette flow, R1/dx 32, and the order check ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: a 132x1x132 slab (y periodic), dx 1 mm, R1 32 mm turning at Omega R1 = 0.1 m/s inside a fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, visc_sweeps 8
measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = 0.0022
reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)
tolerance: <= 0.054 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: closed faces off their rigid velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the mean u_theta over the measured cells = 0.03702 m/s
reference: positive, the rotation's sense
tolerance: > 0 (the cylinder turns the air its own way)
verdict: within
measure: the last step's relative pressure residual = 4.167e-06
reference: a converged solve (the step's fixed point)
tolerance: <= 1e-04 (kResidualAnnounce)
verdict: within
measure: e(32) / e(16) = 0.429
reference: e(16) = 0.0052, the R1/dx 16 record
tolerance: none (report only: e(16) < 0.011; above it, <= 0.7)
verdict: report only
```

```text
=== validation record: planar Couette, plates a quarter cell off the faces ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: 4x16x1 (x, z periodic), dx 1 mm, plates at y = 3.25 dx (still) and 12.75 dx (U 0.1 m/s), nu 1e-3, h 1 ms, 45 steps (half a diffusion time) from the exact profile, n_v 4, visc_sweeps 8
measure: max |u - u_exact| / U, with the wall links = 5.624e-08
reference: 0 (the linear profile is the discrete operator's fixed point)
tolerance: <= 1e-5 (TD-2: measured 5.6e-08 on MSVC 1943 release, rounding)
verdict: within
measure: max |u - u_exact| / U, closed faces only (the negative control) = 6.459e-02
reference: the linear profile, missed: each wall sits 0.75 dx too far
tolerance: > 1e-5 (must exceed the band)
verdict: within
```

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S2 circular Couette flow, R1/dx 16", master c7b3c06 -> P2c, every verdict unchanged: the last step's relative pressure residual: 1.715e-06 -> 1.450e-06.

```text
=== validation record: S2 circular Couette flow, R1/dx 16 ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: a 68x1x68 slab (y periodic), dx 1 mm, R1 16 mm turning at Omega R1 = 0.1 m/s inside a fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, visc_sweeps 8
measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = 0.0052
reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)
tolerance: <= 0.11 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: closed faces off their rigid velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the mean u_theta over the measured cells = 0.03719 m/s
reference: positive, the rotation's sense
tolerance: > 0 (the cylinder turns the air its own way)
verdict: within
measure: the last step's relative pressure residual = 1.450e-06
reference: a converged solve (the step's fixed point)
tolerance: <= 1e-04 (kResidualAnnounce)
verdict: within
```

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S2 circular Couette flow, R1/dx 32, and the order check", master c7b3c06 -> P2c, every verdict unchanged: the last step's relative pressure residual: 4.167e-06 -> 4.843e-06.

```text
=== validation record: S2 circular Couette flow, R1/dx 32, and the order check ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: a 132x1x132 slab (y periodic), dx 1 mm, R1 32 mm turning at Omega R1 = 0.1 m/s inside a fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, visc_sweeps 8
measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = 0.0022
reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)
tolerance: <= 0.054 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: closed faces off their rigid velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: the mean u_theta over the measured cells = 0.03702 m/s
reference: positive, the rotation's sense
tolerance: > 0 (the cylinder turns the air its own way)
verdict: within
measure: the last step's relative pressure residual = 4.843e-06
reference: a converged solve (the step's fixed point)
tolerance: <= 1e-04 (kResidualAnnounce)
verdict: within
measure: e(32) / e(16) = 0.429
reference: e(16) = 0.0052, the R1/dx 16 record
tolerance: none (report only: e(16) < 0.011; above it, <= 0.7)
verdict: report only
```
