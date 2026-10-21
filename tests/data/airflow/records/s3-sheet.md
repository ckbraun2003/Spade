# S3, a sheet across a channel -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; bands in `docs/design/test-docs/2026-10-13-solids-tolerances.md`). A sheet a quarter cell thick, on an even and an odd face plane. One projection from a cold start: exact in 1D.

```text
=== validation record: S3 sheet across a channel, plane 16 (even) ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 7.987e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000002
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the solve's final relative residual = 2.551e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: S3 sheet across a channel, plane 17 (odd) ===
commit: a5a1b4c (physics/airflow-solids)
date: 2026-10-20
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 4.783e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000001
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the solve's final relative residual = 1.913e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: S3 sheet across a channel, plane 16 (even) ===
commit: f4ddf75
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 7.987e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000002
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: P3's force on the sheet F h / (rho U L A) from wall_load() = 0.968752
reference: 1 (exact in 1D; the scheme reads each side half a cell off: 1 - dx/L)
tolerance: 1 +/- 0.033 (test-docs/2026-10-13-solids-tolerances.md, S3)
verdict: within
measure: the solve's final relative residual = 2.551e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: S3 sheet across a channel, plane 17 (odd) ===
commit: f4ddf75
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 4.783e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000001
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: P3's force on the sheet F h / (rho U L A) from wall_load() = 0.968751
reference: 1 (exact in 1D; the scheme reads each side half a cell off: 1 - dx/L)
tolerance: 1 +/- 0.033 (test-docs/2026-10-13-solids-tolerances.md, S3)
verdict: within
measure: the solve's final relative residual = 1.913e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```
