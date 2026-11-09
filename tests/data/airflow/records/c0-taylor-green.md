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

B5.1, 2026-10-26 (the two-level reduction order: blocks of 256 tiles on both twins, 7bdbc8c; Cameron's decision, backlog 45d11fe). "C0 Taylor-Green vortex, 64^3", 20c3bb1 -> B5.1, every verdict unchanged: kinetic energy per unit mass at t = 9.523 (the first step at or past 9.5): 0.074987 -> 0.074988.

```text
=== validation record: C0 Taylor-Green vortex, 64^3 ===
commit: 7bdbc8c
date: 2026-10-26
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
measure: kinetic energy per unit mass at t = 9.523 (the first step at or past 9.5) = 0.074988
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
measure: kinetic energy per unit mass at t = 10.014 (the first step at or past 10.0) = 0.070127
reference: none (report only; DNS data enter at V3, spec §11.6)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C0 Taylor-Green, the 2D mode's exact decay, 32x32x1, u_lat 0.1, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at c38c4878 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 1 thread(s)
config: 32x32x1 periodic slab (a 2D mode in a box of side 2 pi L, L = N / 2 pi), D3Q27 recursive-regularized to 3rd order, WALE off, u_lat 0.1 (Ma 0.173), nu_lat 0.01 (tau 0.53), T = L^2 / (4 nu) = 648 steps, rho from the analytic pressure, populations at the equilibrium of (rho, u), fp32; step under the flush scope, 1 thread(s); final-f digest 0x409a431509491d7d
measure: |nu_eff / nu - 1| = 0.00746
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: <= 0.11 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: nu_eff / nu - 1, signed = 0.00746
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: rate = 0.964 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C0 Taylor-Green, the 2D mode's exact decay, 64x64x1, u_lat 0.05, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 thread(s)
config: 64x64x1 periodic slab (a 2D mode in a box of side 2 pi L, L = N / 2 pi), D3Q27 recursive-regularized to 3rd order, WALE off, u_lat 0.05 (Ma 0.087), nu_lat 0.01 (tau 0.53), T = L^2 / (4 nu) = 2594 steps, rho from the analytic pressure, populations at the equilibrium of (rho, u), fp32; step under the flush scope, 6 thread(s); final-f digest 0xad0d7e766a4a04e3; the gradual-underflow run (SPADE_LBM_GRADUAL=1) of the same configuration: digest 0xad0d7e766a4a04e3, equal
measure: |nu_eff / nu - 1| = 0.00179
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: <= 0.027 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: e(64) / e(32), the order check, reported not checked (e(32) 0.00746 is under a tenth of its band, 0.011) = 0.240
reference: diffusive scaling: N doubles, u_lat halves, tau kept
tolerance: none (report only)
verdict: report only
measure: nu_eff / nu - 1, signed = 0.00179
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: rate = 4.555 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C0 Taylor-Green, the 2D mode's Mach series, 64x64x1, u_lat 0.1 to 0.3, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at c38c4878 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 1 thread(s)
config: 64x64x1 periodic slab, D3Q27 recursive-regularized to 3rd order, WALE off, nu_lat 0.01 (tau 0.53), T 2594 steps, u_lat 0.1, 0.15, 0.2, 0.25, 0.3 (Ma 0.173 to 0.520) against the u_lat 0.05 run, fp32, flush scope on, 1 thread each; final-f digests 0.1 0x9d87fae5e1c1655b, 0.15 0x489cc29f877ab827, 0.2 0x840963bcbf7b1aa7, 0.25 0x99f80d1a9149e8b3, 0.3 0xee3a7056dbfb0b03
measure: nu_eff / nu - 1 at u_lat 0.1 (Ma 0.173), signed = -0.00220
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: the Ma^2 coefficient at u_lat 0.1: the difference from the u_lat 0.05 run over the difference in Ma^2 = -0.178
reference: the note's allowance: a coefficient above 1 in size would mean the allowances are wrong
tolerance: none (report only)
verdict: report only
measure: nu_eff / nu - 1 at u_lat 0.15 (Ma 0.260), signed = -0.00882
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: the Ma^2 coefficient at u_lat 0.15: the difference from the u_lat 0.05 run over the difference in Ma^2 = -0.177
reference: the note's allowance: a coefficient above 1 in size would mean the allowances are wrong
tolerance: none (report only)
verdict: report only
measure: nu_eff / nu - 1 at u_lat 0.2 (Ma 0.346), signed = -0.01777
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: the Ma^2 coefficient at u_lat 0.2: the difference from the u_lat 0.05 run over the difference in Ma^2 = -0.174
reference: the note's allowance: a coefficient above 1 in size would mean the allowances are wrong
tolerance: none (report only)
verdict: report only
measure: nu_eff / nu - 1 at u_lat 0.25 (Ma 0.433), signed = -0.02985
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: the Ma^2 coefficient at u_lat 0.25: the difference from the u_lat 0.05 run over the difference in Ma^2 = -0.176
reference: the note's allowance: a coefficient above 1 in size would mean the allowances are wrong
tolerance: none (report only)
verdict: report only
measure: nu_eff / nu - 1 at u_lat 0.3 (Ma 0.520), signed = -0.04347
reference: the exact decay K(t) = K(0) exp(-4 nu t / L^2) (Taylor 1923)
tolerance: none (report only)
verdict: report only
measure: the Ma^2 coefficient at u_lat 0.3: the difference from the u_lat 0.05 run over the difference in Ma^2 = -0.172
reference: the note's allowance: a coefficient above 1 in size would mean the allowances are wrong
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C0 Taylor-Green vortex, Re 1600, 64^3, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 64^3 periodic 2 pi box, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.0006366 (tau 0.50191), Re 1600, T t* 20 (2037 steps), sampled every 5 steps (dt* 0.0491), rho from the analytic pressure, fp32; step under the flush scope, 6 thread(s); final-f digest 0x9b88aa443975d18f; the gradual-underflow run (SPADE_LBM_GRADUAL=1) of the same configuration: digest 0x9b88aa443975d18f, equal
measure: peak epsilon* / DNS peak (peak 0.011767 at t* 8.492; DNS 0.012858 at t* 8.97) = 0.9152
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3)
tolerance: none (report only)
verdict: report only
measure: t* of the peak / DNS t* = 0.9467
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3)
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 1 (the DNS's 0.000519) = 0.000537
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 2 (the DNS's 0.000708) = 0.001081
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 3 (the DNS's 0.001127) = 0.001779
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 4 (the DNS's 0.002066) = 0.003729
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 5 (the DNS's 0.004127) = 0.007699
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 6 (the DNS's 0.005531) = 0.009179
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 7 (the DNS's 0.007365) = 0.009889
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 8 (the DNS's 0.010373) = 0.010873
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 9 (the DNS's 0.012855) = 0.010777
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 10 (the DNS's 0.011272) = 0.009672
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 11 (the DNS's 0.010098) = 0.008557
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 12 (the DNS's 0.008361) = 0.006417
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 13 (the DNS's 0.006336) = 0.005284
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 14 (the DNS's 0.005424) = 0.004349
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 15 (the DNS's 0.004332) = 0.003814
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 16 (the DNS's 0.003622) = 0.003322
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 17 (the DNS's 0.003228) = 0.002878
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 18 (the DNS's 0.002749) = 0.002399
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 19 (the DNS's 0.002249) = 0.002061
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.329 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: C0 Taylor-Green vortex, Re 1600, 128^3, LBM ===
commit: 9449f83b (physics/lbm-l1, squashed; run at 27253c5 with the same arithmetic)
date: 2026-11-06
path: CPU reference, MSVC 1943, release, 6 threads (WorkerPool)
config: 128^3 periodic 2 pi box, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.001273 (tau 0.50382), Re 1600, T t* 20 (4074 steps), sampled every 10 steps (dt* 0.0491), rho from the analytic pressure, fp32; step under the flush scope, 6 thread(s); final-f digest 0xbf93695782652bd4
measure: |peak epsilon* / DNS peak - 1| (peak 0.011979 at t* 8.787; DNS 0.012858 at t* 8.97) = 0.0684
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3)
tolerance: <= 0.10 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: |t* of the peak / DNS t* - 1| = 0.0204
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3)
tolerance: <= 0.05 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: epsilon* at t* = 1 (the DNS's 0.000519) = 0.000538
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 2 (the DNS's 0.000708) = 0.000742
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 3 (the DNS's 0.001127) = 0.001260
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 4 (the DNS's 0.002066) = 0.002447
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 5 (the DNS's 0.004127) = 0.005416
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 6 (the DNS's 0.005531) = 0.006661
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 7 (the DNS's 0.007365) = 0.008427
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 8 (the DNS's 0.010373) = 0.011409
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 9 (the DNS's 0.012855) = 0.011654
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 10 (the DNS's 0.011272) = 0.009984
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 11 (the DNS's 0.010098) = 0.008692
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 12 (the DNS's 0.008361) = 0.006536
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 13 (the DNS's 0.006336) = 0.005279
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 14 (the DNS's 0.005424) = 0.004534
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 15 (the DNS's 0.004332) = 0.004028
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 16 (the DNS's 0.003622) = 0.003597
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 17 (the DNS's 0.003228) = 0.003254
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 18 (the DNS's 0.002749) = 0.002940
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: epsilon* at t* = 19 (the DNS's 0.002249) = 0.002563
reference: spectral DNS at Re 1600 (tests/data/airflow/tgv1600_spectral.hpp, UCL's pseudo-spectral solution, grid-converged on 512^3) at that t*
tolerance: none (report only)
verdict: report only
measure: rate, 6 threads, shared box = 3.652 MLUPS
reference: none (the lattice's own cost)
tolerance: none (report only)
verdict: report only
```

One run on the device of record, in lock step with the CPU (tests/test_gpu_lbm.cpp, GpuLbmCase.OneCaseInLockStepWithTheCpu): both start from C0b's field, step together, and every 5 steps the device's populations are read back and compared with the CPU's, word for word. The first run at 20d98cc8 printed the same digest in 133.0 s; the rerun below, at d39efd2f, took 186.5 s (the box was shared).

```text
=== validation record: C0 Taylor-Green vortex, Re 1600, 64^3, LBM, on the device in lock step with the CPU ===
commit: physics/lbm-gpu-t5 (T5; run at d39efd2f, SPADE_LBM_VALIDATION=1 SPADE_LBM_GPU_CASE=c0b:64)
date: 2026-11-09
path: the device of record (the chain of lbm_macro and lbm_collide_stream, the staged lbm_f/lbm_u buffers) in lock step with the CPU reference, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes; CPU MSVC 1943, release, 6 threads (WorkerPool)
config: 64^3 periodic 2 pi box, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.0006366 (tau 0.50191), Re 1600, T t* 20 (2037 steps), the populations compared every 5 steps (408 samples), fp32; the CPU step under the flush scope; final-f digest 0x9b88aa443975d18f (the CPU record's digest above, equal)
measure: words of f differing from the CPU at the 408 sample steps = 0
reference: the CPU reference's populations, word for word (TD-14: bit equality is for numbers, not NaN payloads, so the CPU field is checked finite at each sample)
tolerance: exact (TD-14)
verdict: within
measure: kinetic energy at its largest, over its first, at the samples = 1.0000
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
```
