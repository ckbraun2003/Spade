# L1, the LBM lattice: the stability map -- validation records

One block per u_lat, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`). Report only: the map decides the collision rung, and its points carry no tolerance (`docs/design/test-docs/2026-11-03-l1-lbm-tolerances.md`, "The stability map"). The decision it fed is L33 (the lead): R0, RR3 + WALE (C_w 0.5), no fallback. The coverage table and the max-|u| rule that follow from it are in `docs/design/physics/plans/2026-11-04-l1-lbm-design.md`, section 8.

How to read a point: `steps held` is the last step whose sample (every 5th step) still had K* finite and at most 1.05 K*(0), of the horizon (t* 20); a point that held to the horizon reads `of` its own horizon. `K* / K*(0)` is at the last sample that held. The MLUPS beside each point is one single-thread scalar process running with up to five others on the same box: concurrent, not a cost record (the cost record is the last block).

Run in five tickets, one at a time (`spade-physics-lbm-map-b1` to `b5`, jobrun, `build-ninja/release/bin/spade_tests.exe --gtest_filter=LbmMap.*`, `SPADE_LBM_VALIDATION=1 SPADE_LBM_MAP=u:nu`): b1 the 0.025 and 0.05 rows (2797 s), b2 u_lat 0.10 and 0.15 (1198 s), b3 u_lat 0.20 to 0.40 and the 0.254 row (940 s), b4 u_lat 0.275 (300 s, after the print was widened to three places; the arithmetic is unchanged), b5 u_lat 0.275 at the three other nu_lat (280 s; L33b). That block is a second identity beside the first so the reader keeps both; together they are all five nu_lat at 0.275.

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.025 (Ma 0.043), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.025 (Ma 0.043), to t* 20 (8149 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 1.5e-4 (tau - 1/2 = 0.000450000021), of 8149 = 8149
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1.5e-4 = 0.1332
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1.5e-4, concurrent with up to 5 other points, not a cost record = 0.765 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 3e-4 (tau - 1/2 = 0.000900000043), of 8149 = 8149
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 3e-4 = 0.1323
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 3e-4, concurrent with up to 5 other points, not a cost record = 0.773 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.05 (Ma 0.087), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.05 (Ma 0.087), to t* 20 (4074 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 9.4e-6 (tau - 1/2 = 2.81999992e-05), of 4074 = 4074
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 9.4e-6 = 0.1513
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 9.4e-6, concurrent with up to 5 other points, not a cost record = 0.770 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.10 (Ma 0.173), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.10 (Ma 0.173), to t* 20 (2037 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 2037 = 2037
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.1767
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.731 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 2037 = 2037
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.1767
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.745 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 2037 = 2037
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.1764
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.740 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 2037 = 2037
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.1732
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.745 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 2037 = 2037
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1370
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.733 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.15 (Ma 0.260), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.15 (Ma 0.260), to t* 20 (1358 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 1358 = 1358
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.1787
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.736 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 1358 = 1358
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.1787
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.736 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 1358 = 1358
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.1787
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.760 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 1358 = 1358
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.1793
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.757 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 1358 = 1358
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1539
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.757 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.20 (Ma 0.346), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.20 (Ma 0.346), to t* 20 (1019 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 1019 = 1019
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.1766
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.735 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 1019 = 1019
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.1766
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.735 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 1019 = 1019
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.1765
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.742 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 1019 = 1019
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.1764
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.744 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 1019 = 1019
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1676
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.741 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.25 (Ma 0.433), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.25 (Ma 0.433), to t* 20 (815 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.1775
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.736 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.1775
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.735 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.1772
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.740 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.1750
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.728 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1724
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.736 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.254 (Ma 0.440), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.254 (Ma 0.440), to t* 20 (802 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 4.2e-6 (tau - 1/2 = 1.26000004e-05), of 802 = 802
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 4.2e-6 = 0.1796
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 4.2e-6, concurrent with up to 5 other points, not a cost record = 0.733 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.275 (Ma 0.476), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 0ccdd7c, 1 thread, gradual underflow)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.275 (Ma 0.476), to t* 20 (741 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 741 = 741
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.1866
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.663 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 741 = 741
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.1859
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.656 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.30 (Ma 0.520), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.30 (Ma 0.520), to t* 20 (679 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 679 = 320
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.7067
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.734 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 679 = 320
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.7050
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.720 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 679 = 320
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.7027
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.739 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 679 = 350
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.6229
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.742 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 679 = 679
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1730
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.746 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.35 (Ma 0.606), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.35 (Ma 0.606), to t* 20 (582 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 582 = 235
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.7628
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.741 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 582 = 235
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.8001
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.731 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 582 = 235
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.7380
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.730 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 582 = 245
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.6951
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.736 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 582 = 582
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1726
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.763 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.40 (Ma 0.693), LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 5d385e4 with 1 thread under gradual underflow, before the thread split and the flush scope; the edge-point table below shows the flush moved no bit at u_lat 0.275 and 0.30)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.40 (Ma 0.693), to t* 20 (509 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 509 = 170
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.9726
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 0, concurrent with up to 5 other points, not a cost record = 0.729 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 509 = 175
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.8596
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 5 other points, not a cost record = 0.726 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-5 (tau - 1/2 = 2.99999992e-05), of 509 = 175
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-5 = 0.8421
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-5, concurrent with up to 5 other points, not a cost record = 0.725 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 509 = 180
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.9453
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 5 other points, not a cost record = 0.754 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 509 = 280
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.5691
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 5 other points, not a cost record = 0.764 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.275 (Ma 0.476), the three further nu_lat, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at the build of 0ccdd7c, ticket spade-physics-lbm-map-b5-7bb3b1a, 1 thread, gradual underflow)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.275 (Ma 0.476), to t* 20 (741 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step; with the u_lat 0.275 block above (nu_lat 0 and 1e-5) every nu_lat of the map is run at this u_lat (L33b)
measure: steps held at nu_lat 1e-6 (tau - 1/2 = 2.99999999e-06), of 741 = 741
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-6 = 0.1865
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-6, concurrent with up to 2 other points, not a cost record = 0.703 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-4 (tau - 1/2 = 0.000299999992), of 741 = 741
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-4 = 0.1806
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-4, concurrent with up to 2 other points, not a cost record = 0.702 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: steps held at nu_lat 1e-3 (tau - 1/2 = 0.00300000014), of 741 = 741
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 1e-3 = 0.1729
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: single-thread rate at nu_lat 1e-3, concurrent with up to 2 other points, not a cost record = 0.704 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

## The flush scope at the edge points (2026-11-06)

The runs above were made with the CPU reference stepping under gradual underflow, before the step was put under the flush scope (FTZ|DAZ, as the device of record flushes; GPU twin plan, T0). The flush moved the final populations of the walled cavity C1 Re 100 (subnormals in the decayed corner fields after 64000 steps) and moved nothing in C0a 64 or C0b 64 (equal digests). The map's field is C0b's, so the edge points were run both ways, one line each: u_lat 0.275 and 0.30 at every nu_lat, the scoped step (`spade-physics-lbm-map-b6`) against the unscoped one (`SPADE_LBM_GRADUAL=1`, `b7`), 1 thread each, final-f digest (FNV over the populations' bits and the step count). All ten are equal and the held counts are the same as above, so the interior points are not re-run and the records above stand.

| u_lat | nu_lat | steps held (both ways) | digest, scoped | digest, gradual | |
|---|---|---|---|---|---|
| 0.275 | 0 | 741 of 741 | `0xcfb35055c50971f0` | `0xcfb35055c50971f0` | equal |
| 0.275 | 1e-6 | 741 of 741 | `0xaab4fb8c262a6a98` | `0xaab4fb8c262a6a98` | equal |
| 0.275 | 1e-5 | 741 of 741 | `0x2b00ec7fd77f4870` | `0x2b00ec7fd77f4870` | equal |
| 0.275 | 1e-4 | 741 of 741 | `0x982a8bbad74af0a8` | `0x982a8bbad74af0a8` | equal |
| 0.275 | 1e-3 | 741 of 741 | `0x884f275f12606928` | `0x884f275f12606928` | equal |
| 0.30 | 0 | 320 of 679 | `0xf000de6e05edff01` | `0xf000de6e05edff01` | equal |
| 0.30 | 1e-6 | 320 of 679 | `0xde17b7f939896149` | `0xde17b7f939896149` | equal |
| 0.30 | 1e-5 | 320 of 679 | `0x03919e0a86fef109` | `0x03919e0a86fef109` | equal |
| 0.30 | 1e-4 | 350 of 679 | `0x49aafdd4ba3c80b3` | `0x49aafdd4ba3c80b3` | equal |
| 0.30 | 1e-3 | 679 of 679 | `0x3c0effde7190d5fa` | `0x3c0effde7190d5fa` | equal |

## The cost record

The R0 row for `lbm_seconds` (Interface's offline-tier note, L30). One process, one thread, scalar, on the map's point u_lat 0.10, nu_lat 1e-5, the same code the table is built on.

```text
=== validation record: L1 cost record, R0, 64^3 Taylor-Green, u_lat 0.100 nu_lat 1e-5, solo probe, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 0ccdd7c, 1 thread, gradual underflow)
date: 2026-11-05
path: CPU reference, MSVC 1943, release, 1 thread, scalar
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.100 (Ma 0.173), nu_lat 1e-5, to t* 20 (2037 steps), rho from the analytic pressure, fp32; conditions: solo in Spade's queue (ticket spade-physics-lbm-solo-r0-0ccdd7c, both heavy slots free at its start and end), Kat's gate jobs ran on the box (3 waiting at the start, 6 at the end); the 120 B reference without solid flags (124 B with them)
measure: single-thread rate, solo in Spade's queue, Kat's gate jobs on the box = 0.792 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
measure: wall time of the 2037 steps = 674 s
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

The earlier single-thread probes of the cases ran at 0.81 to 0.93 MLUPS (C0a 64 0.925, C0b 64 0.814, C1 Re 100 0.837; `.superpowers/sdd/2026-11-04-lbm/probe-*.log`), so 0.79 is the conservative number. A quiet re-probe (an empty queue, low priority) replaces the row only if it comes out higher (L33a).

## The exact point on the device (2026-11-09)

The L5 point (u_lat 0.196, nu_lat 4.2e-6) stepped on the device of record in lock step with the CPU to t* 20 (tests/test_gpu_lbm.cpp, GpuLbmCase.OneCaseInLockStepWithTheCpu, `map:0.196:4.2e-6`): the populations compared word for word every 5 steps. The first run at 20d98cc8 printed the same digest in 63.8 s; the rerun at d39efd2f took 94.7 s.

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.196, nu_lat 4.2e-6, LBM, on the device in lock step with the CPU ===
commit: physics/lbm-gpu-t5 (T5; run at d39efd2f, SPADE_LBM_VALIDATION=1 SPADE_LBM_GPU_CASE=map:0.196:4.2e-6)
date: 2026-11-09
path: the device of record (the chain of lbm_macro and lbm_collide_stream, the staged lbm_f/lbm_u buffers) in lock step with the CPU reference, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes; CPU MSVC 1943, release, 6 threads (WorkerPool)
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.196 (Ma 0.339), nu_lat 4.2e-6, to t* 20 (1039 steps), rho from the analytic pressure, fp32; the CPU step under the flush scope, the populations compared every 5 steps (208 samples); final-f digest 0x156f2790a0b24740
measure: steps held to t* 20, of 1039 = 1039
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: kinetic energy at its largest, over its first, at the samples = 1.0000
reference: K* finite and <= 1.05 K*(0) at every 5th step (the map's own test)
tolerance: <= 1.05 (test-docs/2026-11-03-l1-lbm-tolerances.md, the map's held test, fixed before the first run)
verdict: within
measure: words of f differing from the CPU at the 208 sample steps = 0
reference: the CPU reference's populations, word for word (TD-14: bit equality is for numbers, not NaN payloads, so the CPU field is checked finite at each sample)
tolerance: exact (TD-14)
verdict: within
```
