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
