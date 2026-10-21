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
