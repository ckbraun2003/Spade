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

```text
=== validation record: C0 Taylor-Green vortex, 64^3 ===
commit: b794356 (physics/airflow-core)
date: 2026-10-12
path: CPU reference, MSVC 1943, release
config: 64^3 periodic 2 pi box, inviscid (the numerics' own dissipation), h = 0.049087, n_v 2
measure: kinetic energy per unit mass at t = 0.000 (the first step at or past 0.0) = 0.125000
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 0.540 (the first step at or past 0.5) = 0.124007
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 1.031 (the first step at or past 1.0) = 0.123229
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 1.522 (the first step at or past 1.5) = 0.122599
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 2.013 (the first step at or past 2.0) = 0.122109
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 2.503 (the first step at or past 2.5) = 0.121745
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 3.043 (the first step at or past 3.0) = 0.121442
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 3.534 (the first step at or past 3.5) = 0.121155
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 4.025 (the first step at or past 4.0) = 0.120747
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 4.516 (the first step at or past 4.5) = 0.119737
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 5.007 (the first step at or past 5.0) = 0.117116
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 5.547 (the first step at or past 5.5) = 0.112690
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 6.038 (the first step at or past 6.0) = 0.109314
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 6.529 (the first step at or past 6.5) = 0.106945
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 7.019 (the first step at or past 7.0) = 0.103703
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 7.510 (the first step at or past 7.5) = 0.098525
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 8.001 (the first step at or past 8.0) = 0.091686
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 8.541 (the first step at or past 8.5) = 0.084797
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 9.032 (the first step at or past 9.0) = 0.079821
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 9.523 (the first step at or past 9.5) = 0.074987
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 10.014 (the first step at or past 10.0) = 0.070127
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
```
