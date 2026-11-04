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

P2c, 2026-10-22 (the coarse faces' plane means in series, 9edcc53; the lead's TD-1 word, 2026-10-22). "S3 sheet across a channel, plane 17 (odd)", master c7b3c06 -> P2c, every verdict unchanged: max |u| / U after the projection: 4.783e-06 -> 6.378e-06; the pressure step dp h / (rho U (L - dx)): 1.000001 -> 1.000000; P3's force on the sheet F h / (rho U L A) from wall_load(): 0.968751 -> 0.968750; the solve's final relative residual: 1.913e-05 -> 2.551e-05.

```text
=== validation record: S3 sheet across a channel, plane 17 (odd) ===
commit: 9edcc53
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 6.378e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000000
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: P3's force on the sheet F h / (rho U L A) from wall_load() = 0.968750
reference: 1 (exact in 1D; the scheme reads each side half a cell off: 1 - dx/L)
tolerance: 1 +/- 0.033 (test-docs/2026-10-13-solids-tolerances.md, S3)
verdict: within
measure: the solve's final relative residual = 2.551e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```

## 2026-11-03: S3 on the device (B8)

GpuFluid, the whole step on the device of record (RTX 3060 Ti, driver 572.83), against the CPU's run of the same box over 32 steps: TD-14 as amended, bit-equal. The step time is report only (the lead, 2026-11-03). Planes 16 and 17, the box as S3ASheetAcrossAChannelStopsItsFlow builds it, run as the whole step (the CPU's S3 record above is one projection). The config line's doubled "solids:" is as printed.

```text
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 16 (even), GpuFluid (G3) against the CPU ===
commit: dfefd3fc (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8-records-dfefd3fc)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 16, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 18.9 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s; its mean speed at step 32 = 9.7494e-09 m/s, the CPU's 9.7494e-09 m/s (within 10% + 1e-04 m/s)`

```text
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 17 (odd), GpuFluid (G3) against the CPU ===
commit: dfefd3fc (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8-records-dfefd3fc)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 17, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 18.0 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s; its mean speed at step 32 = 1.05647e-08 m/s, the CPU's 1.05647e-08 m/s (within 10% + 1e-04 m/s)`

## 2026-11-03: S3 on the device, at the final commit (B8)

The same runs at 7230ec46, rebuilt on MSVC after the config line's fix: it now prints "solids:" once. Bit-equal again over 32 steps on the device of record (RTX 3060 Ti, driver 572.83); the step time is report only. Same identities as the blocks above, so these later blocks are the ones read.

```text
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 16 (even), GpuFluid (G3) against the CPU ===
commit: 7230ec46 (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8f-records-7230ec46)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 16, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 19.5 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s; its mean speed at step 32 = 9.7494e-09 m/s, the CPU's 9.7494e-09 m/s (within 10% + 1e-04 m/s)`

```text
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 17 (odd), GpuFluid (G3) against the CPU ===
commit: 7230ec46 (test-docs/b8-gpu-records; queue ticket spade-testdocs-b8f-records-7230ec46)
date: 2026-11-03
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 17, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 18.0 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them (checks, not measures; the floor is the check other devices run, the lead's ruling 2026-11-03, backlog 145386d9):

- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s; its mean speed at step 32 = 1.05647e-08 m/s, the CPU's 1.05647e-08 m/s (within 10% + 1e-04 m/s)`

## 2026-11-04: L13, the aperture floor (Physics)

build_solids() closes every wall face whose ramp leaves under 5% open (kApertureFloor; the lead's L13), re-measured at the fix's commit with SPADE_AIRFLOW_VALIDATION=1. Every verdict holds; the moved values are the floor's.

```text
=== validation record: S3 sheet across a channel, plane 16 (even) ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
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
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 16 (even), GpuFluid (G3) against the CPU ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 16, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 17.6 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them:

- `the CPU's largest |v| at step 32: 0.0000 m/s`
- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 8.821e-06 m/s against 4 m/s; its mean speed at step 32 = 9.7494e-09 m/s, the CPU's 9.7494e-09 m/s (within 10% + 1e-04 m/s)`

```text
=== validation record: S3 sheet across a channel, plane 17 (odd) ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
path: CPU reference, MSVC 1943, release
config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho 1.225, U 1.0, h 0.0010, one projection, n_v 32 CG iterations from a cold start; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm)
measure: closed faces off their wall velocity = 0
reference: 0 (a closed face carries its wall's velocity)
tolerance: 0, exact (the solids design, section 3)
verdict: within
measure: max |u| / U after the projection = 6.378e-06
reference: 0 (exact in 1D)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: the pressure step dp h / (rho U (L - dx)) = 1.000000
reference: 1 (exact: the stencil is exact for a linear field)
tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)
verdict: within
measure: P3's force on the sheet F h / (rho U L A) from wall_load() = 0.968750
reference: 1 (exact in 1D; the scheme reads each side half a cell off: 1 - dx/L)
tolerance: 1 +/- 0.033 (test-docs/2026-10-13-solids-tolerances.md, S3)
verdict: within
measure: the solve's final relative residual = 2.551e-05
reference: a converged solve
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: B8, the whole step on the device, S3 sheet across a channel, plane 17 (odd), GpuFluid (G3) against the CPU ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
path: GpuFluid (G3), MSVC 1943, release, on NVIDIA GeForce RTX 3060 Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no mode requested, device default (preserve supported: no, flush-to-zero supported: no); fp32 NaN preservation supported: yes
config: S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane 17, rho 1.225, U 1.0 at the start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor 32, cap 32; solids: 1 parts, 0 walls, 1 sheets; sheet: sheet (7.8 mm < 2 dx = 62.5 mm); steps 1 to 32 compared from the initial state; the step time the median of 9 steps after 3 warm-up; the film's 0.50 s not run on this box
measure: the values differing from the CPU's run over steps 1 to 32 (u, v, w and p, unconverged, pressure_iterations, pressure_residual, stalled, capped, the wall load; not compared: fault, the solids) = 0 of 280928
reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)
tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both twins flush)
verdict: within
measure: the first differing step, 0 if none (none differ) = 0
reference: the CPU's run of the same box
tolerance: none (report only)
verdict: report only
measure: the candidate's step time, the median of 9 steps after 3 warm-up = 18.6 ms
reference: none (report only): at the D1 box, Cameron's bound of 190 ms a step with the bits kept (2026-11-03, docs/design/backlog.md) and the plan's estimate of about 95 ms (docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)
tolerance: none (report only)
verdict: report only
```

Beside the block, as the run printed them:

- `the CPU's largest |v| at step 32: 0.0000 m/s`
- `physical: PASS; the CPU's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s (4 x (2 v_h + the stream))`
- `floor: PASS; the candidate's largest |u|, |v| or |w| over steps 1 to 32 = 1.971e-05 m/s against 4 m/s; its mean speed at step 32 = 1.05647e-08 m/s, the CPU's 1.05647e-08 m/s (within 10% + 1e-04 m/s)`
