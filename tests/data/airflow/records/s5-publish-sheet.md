# S5, Publish beside a sheet -- validation records

One block per run, appended, never edited (Test/Docs' format, `docs/design/test-docs/01-verification.md`; bands in `docs/design/test-docs/2026-10-13-solids-tolerances.md`, S5). A velocity linear on each side of a sheet with jumps in u and w across it, published with the sheet's closed faces: a gradient that crossed the sheet would publish |omega| near 70 S.

```text
=== validation record: S5 Publish beside a sheet ===
commit: f4ddf75
date: 2026-10-22
path: CPU reference, MSVC 1943, release
config: 8x16x8 at dx 1 cm, x and z periodic, a sheet's closed v-faces at y = 8 dx; u and w linear on each side with jumps U = 1 m/s across it, S dx = 0.01 U, v = 0; published with the apertures, no step taken
measure: max | |omega| - S | / S over the 128 cells next to the sheet = 0.000e+00
reference: 0 (|omega| = S exactly; a one-sided difference is exact for a linear field)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, S5)
verdict: within
measure: max |q-hat| / S over the same cells = 0.000e+00
reference: 0 (Q = 0 exactly)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, S5)
verdict: within
```

## 2026-11-04: L13, the aperture floor (Physics)

build_solids() closes every wall face whose ramp leaves under 5% open (kApertureFloor; the lead's L13), re-measured at the fix's commit with SPADE_AIRFLOW_VALIDATION=1. Every verdict holds; the moved values are the floor's.

```text
=== validation record: S5 Publish beside a sheet ===
commit: b0903118 (physics/theta-floor; queue ticket spade-physics-theta-records-b090311)
date: 2026-11-04
path: CPU reference, MSVC 1943, release
config: 8x16x8 at dx 1 cm, x and z periodic, a sheet's closed v-faces at y = 8 dx; u and w linear on each side with jumps U = 1 m/s across it, S dx = 0.01 U, v = 0; published with the apertures, no step taken
measure: max | |omega| - S | / S over the 128 cells next to the sheet = 0.000e+00
reference: 0 (|omega| = S exactly; a one-sided difference is exact for a linear field)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, S5)
verdict: within
measure: max |q-hat| / S over the same cells = 0.000e+00
reference: 0 (Q = 0 exactly)
tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, S5)
verdict: within
```
