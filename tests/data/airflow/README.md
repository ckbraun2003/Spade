# Airflow reference data (spec §11.1)

This directory holds every validation case's reference data, with its provenance. Test/Docs reviews each file like a golden before its case's first run. A change to one needs a named cause (`TD-2`).

| Case | File | Source | Method |
|---|---|---|---|
| C0, the Taylor–Green vortex | none. The initial field is closed form: `u = sin x cos y cos z`, `v = −cos x sin y cos z`, `w = 0` | Brachet et al. (1983), J. Fluid Mech. 130 | V1 reports the numerics' own dissipation; DNS data enter at V3 |
| C1, the lid-driven cavity | `ghia1982_cavity.hpp` (Re 100 and 400) | Ghia, Ghia & Shin (1982), J. Comput. Phys. 48, 387–411, doi:10.1016/0021-9991(82)90058-4: Table I (p. 398) and Table II (p. 399) | Read from a scan of the published pages and checked against the scan's OCR layer. Cross-checked with two independent reproductions (the ivan-pi gists; Oceananigans.jl), which agree in every value and coordinate. The Uppsala thesis's 0.05188 (Table II, point 65, Re 400) is a misreading. Point 117, Re 400 (−0.23827) is kept as printed, a suspected typo in the original. Compared by machine with that reading: 102 values, no difference. Re 1000 is added, scan-verified, when V2 needs it |
| C8, the hovering actuator disc | none. Closed form: `v_h = √(T / 2ρA)`; on the axis at `z` below the disc, `v = v_h (1 + z / √(z² + R²))` | Momentum theory; Leishman, *Principles of Helicopter Aerodynamics*, 2nd ed. (2006), ch. 2; this repo's `engine/vehicles/rotor_wake.hpp` §1 | Formula |

`records/` holds the validation records: one file per case, appended to by each run, never edited. Each record is a fenced `text` block in the shape `docs/design/test-docs/01-verification.md` defines.
