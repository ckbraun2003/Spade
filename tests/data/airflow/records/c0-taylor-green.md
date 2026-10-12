# C0, the Taylor-Green vortex -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). Report only: V1 records the numerics' own dissipation (inviscid); DNS data enter at V3.

```text
=== validation record: C0 Taylor-Green vortex, 64^3 ===
commit: 81a87da (physics/airflow-core)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 64^3 periodic 2 pi box, inviscid (the numerics' own dissipation), h = 0.049087, n_v 2
measure: kinetic energy per unit mass at t = 0.00 = 0.125000
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 2.50 = 0.121745
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 5.00 = 0.117116
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 7.50 = 0.098525
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 10.00 = 0.070127
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
```
