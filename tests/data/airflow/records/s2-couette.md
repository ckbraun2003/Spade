# S2, circular Couette flow -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; bands in `docs/design/test-docs/2026-10-13-solids-tolerances.md`). S2 checks the solution, not the wall treatment: at R1/dx 16 it passes with the viscous wall links (0.0052) and also without them (0.0602, P2 Task 8's mutation). The planar Couette check is the one that tells the two treatments apart, with its negative control in the same block. R1/dx 32 and the order check are opt-in (`SPADE_AIRFLOW_VALIDATION`). S2 skips in debug builds under TD-5's "debug: runs in release" terms (100 s in debug, 15 s in release).

```text
=== validation record: S2 circular Couette flow, R1/dx 16 ===
commit: 85364c3 (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: a 68x1x68 slab (y periodic), dx 1 mm, R1 16 mm turning at Omega R1 = 0.1 m/s inside a fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, visc_sweeps 8
measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = 0.0052
reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)
tolerance: <= 0.11 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: closed faces carry the rigid velocity = yes; mean u_theta = 0.03719 m/s (the rotation's sense: positive)
```

```text
=== validation record: S2 circular Couette flow, R1/dx 32, and the order check ===
commit: 85364c3 (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
measure: max |u_theta - u_theta,exact| / (Omega R1) at R1/dx 32 = 0.0022
tolerance: <= 0.054 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: e(32) / e(16) = 0.429
tolerance: <= 0.7 (an observed order of at least 0.5; reported only if e(16) < 0.011)
verdict: report only
```

```text
=== validation record: planar Couette, plates a quarter cell off the faces ===
commit: 85364c3 (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: 4x16x1 (x, z periodic), dx 1 mm, plates at y = 3.25 dx (still) and 12.75 dx (U 0.1 m/s), nu 1e-3, h 1 ms, 45 steps (half a diffusion time) from the exact profile
measure: max |u - u_exact| / U, with the wall links = 5.624e-08
negative control: the same, closed faces only (links cleared) = 6.459e-02
tolerance: <= 1e-5 with the links (TD-2: measured 5.6e-08, rounding); the control must exceed it
verdict: within; control fails, as it must
```
