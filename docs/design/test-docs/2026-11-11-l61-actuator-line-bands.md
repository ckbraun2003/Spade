# L61 / P4, the actuator line's validation: V1 to V4's bands, fixed before the first run

**Owner:** Test/Docs. **Status:** written 2026-11-11, before any V1 to V4 run. Physics' proposals (`p4-actuator-line-design.md` §7, in its worktree) are taken term by term: kept where a term supports them, changed where one does not.
**For:** V1 (a grid-free BEMT on an estimated polar against UIUC), V2 (one rotor hovering in a grid box at R/dx 19, 25 and 38), V3 (the Phantom 3 against NASA's hover) and V4 (D1, report only). The bands bind the runs, not the unit tests.
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. It is not loosened after a run without a lead decision.

## What Physics should know first

1. **The estimated polar jumped at stall, and no longer does.** The design note's formula gave C_l 1.34 at α = 11° and sin 2α = 0.375 just above it. Physics' shipped polar (`engine/physics/airflow/blade.hpp`, physics/line @ 87571693) is attached to 11° and blended linearly in α over 14° more to the flat plate, so it is continuous. The sensitivities below were first run on the old polar and re-run on the shipped one (this note's numbers are the shipped polar's).
2. **V3 at ±10% is not reachable by a correct line.** The library's own measured table puts the 9.4 in stand-in at 2.995 N at 5300 rpm (ρ 1.225), 3.9% under NASA's 3.115 N a rotor. That leaves 6.1% for the whole chain. V3's band is the sum of its terms (below).
3. **The held-out J range ends at 0.581.** The table's last measured point is J 0.581 (`first_measured_j` 0.113, `last_measured_j` 0.581). J 0.6 has no data, and J 0.5's band is wider than half its value (below).
4. **The 0.78 R contraction and "2 v_h at 1.5 R" are not momentum theory's.** The ideal far wake has 2 v_h and 0.707 R. I cannot source 0.78 R, so these two are report only.

## The reference and its uncertainty

- **UIUC's data.** Brandt and Selig, AIAA 2011-1255 (the publisher's open copy, read 2026-11-11; `.agents/refs/brandt-selig-2011/`). The paper states **no numeric uncertainty**: it points to Brandt's 2005 thesis, which I have not read. It states significant Reynolds-number effects, performance falling at lower rpm.
- **So the reference term is measured, not quoted:** the library's own step between UIUC's static file and its run at J → 0: **+2.7% in C_T and −1.5% in C_P** (10x5; the 9x4.5's is about 10%). Static and run are two test modes of the same rig.
- **The measured configuration** is the 10 in prop at 5000 rpm (`run_rpm`), ρ 1.225. V1 runs there, not at the 9.4 in scale of V3.
- **The static point** is the library's shipped one: C_T 0.09636, C_Q 0.005870 (C_Q = C_P / 2π). Its J table is at J = k/20, resampled from the run. Below the first measured point it interpolates to the static point, so J < 0.113 is never a held-out point.

## The polar's own spread (computed for this note, relative only)

A hover BEMT written for this note, on the library's `uiuc_apce_10x5` stations, D 0.254 m, 5000 rpm, the estimated polar as shipped (the blend above). **Only relative changes were printed, never an absolute C_T or C_Q and never a comparison with UIUC's data**, so no band comes from the model's error (`.agents/refs/l61-sensitivity/`).

| Change to the polar (static point) | dC_T | dC_Q |
|---|---|---|
| lift slope 5.5 → 2π (thin-plate theory) | +10.8% | +13.7% |
| lift slope 5.5 → 4.4 (−20%, my bracket) | −15.9% | −18.6% |
| α₀ −3° → −1° / −5° | −14.5% / +14.9% | −17.1% / +19.3% |
| C_d × 0.7 / × 1.5 | +0.3% / −0.4% | −9.0% / +15.1% |
| Prandtl tip loss on, against off | −3.5% | −0.8% |

An estimated polar's plausible spread is about ±15% in thrust and ±19% or more in torque. So **V1's static band is a decision threshold for calibration, not a validation**: a pass would be luck. The validation is the held-out J.

## V1: the BEMT against UIUC's 10x5

**V1a, the static point (a trigger).** C_T within ±10% and C_Q within ±15% of the static point (Physics' numbers, kept as the threshold). Inside both, the estimated polar stands. Outside either, Physics calibrates α₀ and the drag scale to the static point, **once**, two parameters for two numbers, and the record carries the calibrated values and says "calibrated". V1a then passes by construction.

**V1b, the held-out J (the test).** At J 0.2, 0.3 and 0.4 (inside the measured range, held out of any calibration), |C_T(J) − C_T,UIUC(J)| and |C_Q(J) − C_Q,UIUC(J)| are each at most the band below, as a share of the *static* UIUC value. A share of the local value is useless as C_T falls to zero.

The terms, per J:

- **The slope:** with α₀ and the drag scale re-matched so the static point equals the nominal's, a polar slope anywhere from 4.4 to 2π moves the J sweep by up to the values below (the −20% side; the 2π side is 2.4, 3.9 and 5.8% in C_T). This is the one polar freedom a static point cannot fix. The 4.4 is my bracket, a judgment, and the record says so.
- **The reference:** 2.7% (C_T) or 1.5% (C_Q) of the static value.
- **If the polar stood** (no calibration), the static band is added.

| J | slope term, C_T | slope term, C_Q | band after calibration, C_T / C_Q | band if the polar stood, C_T / C_Q |
|---|---|---|---|---|
| 0.2 | 3.6% | 4.9% | **6.3% / 6.4%** | 17% / 22% |
| 0.3 | 6.0% | 8.8% | **8.7% / 11%** | 19% / 26% |
| 0.4 | 8.8% | 13.7% | **12% / 16%** | 22% / 31% |

- **J 0.5 is report only.** The slope term there is 12.0% and 19.7% of the static value, but UIUC's C_T at J 0.5 is 0.029, 30% of the static value. The band would cover over half the quantity.
- **J 0.6 has no UIUC point** (last measured 0.581). **The efficiency** J C_T / C_P is report only: it is built from two banded quantities.
- **A second calibration is refused:** a miss at J 0.2 to 0.4 after calibrating is a finding about the polar's shape, not a reason to refit.

## V2: one rotor hovering in a grid box

CPU at R/dx 19 and 25, the GPU at 38 (4, 3 and 2 mm for D1's rotor). Compared at one rotor, one disc loading, the same blade stations and the same polar (calibrated by V1 if it was). **The comparator is the BEMT with Prandtl tip loss**, the standard hover model. The line has no tip correction, but its grid carries the tip vortex, so the tip loss belongs in the comparator. If Physics' BEMT has none, the thrust band's centre moves by −3.5% and the power's by −0.8%, and the record says which BEMT it used.

| Measure | Band | Built from |
|---|---|---|
| Line thrust against the BEMT | **within ±10%** | Physics' number, kept. What I can derive is small: the tip-loss term is the 3.5% above, removed by the comparator. The rest (the kernel's smoothing, eps/c of 0.5 to 0.9 against the 0.25 optimum Physics cites, so the load reads low; the grid; the root) cannot be separated before a run. The R/dx sweep is what measures it |
| Line power (torque × ω) against the BEMT | **within ±15%** | Physics' number, kept. Torque is the more sensitive: the drag term moves it ±10 to 17% on its own |
| Momentum flux through the plane 1.5 R below, over the line's own thrust | **within ±10%** | steady momentum theory: with ambient pressure on the plane, the flux ρ ∫ w² dA equals the thrust, centre 1. The width is the L52 calibration's 0.089 at a similar plane, rounded up (`2026-11-10-sgs-film-bands.md`). It does not use the BEMT |
| Figure of merit T^1.5 / (√(2ρA) P) | report only | built from the two banded quantities: the worst-case band, 1.5 × 10% + 15% = 30%, tells nothing. The UIUC static value is 0.573 |
| Far-wake axial speed, wake contraction | report only | the ideal values are 2 v_h and 0.707 R; Physics' 0.78 R is unsourced to me |

- **The window.** The mean over at least 10 R/v_h, starting at 20 R/v_h or later (v_h from the BEMT), with the start-up excluded. If the last two such windows' thrusts differ by more than 2.5% (a quarter of the band), the run is **unsettled** and runs longer; the band is not widened.
- **The R/dx rule.** The rule stays at 20 unless the R/dx 19 run is inside the bands above and its thrust is within **5%** of the R/dx 38 run's (power within 7.5%): half each band, so the resolution error is small beside the model's. R/dx 25 gets the same two tests; a miss there is a finding for the line, not for the rule.

## V3: the Phantom 3 against NASA

**The line's thrust at 5300 rpm on `uiuc_apce_10x5` at D 9.4 in is within ±26% of 3.115 N a rotor** (12.46 N over four). Worst-case terms, summed:

| Term | Value | Built from |
|---|---|---|
| V2's line against the BEMT | 10% | the band above |
| V1a's polar against UIUC's static point | 10% | the threshold, if the polar stood. After calibration this is the reference step, 2.7%, so the sum falls to 18.4% |
| The stand-in table against NASA | 3.9% | the library's measured C_T at 5300 rpm is 2.995 N, 3.9% under 3.115 N |
| NASA's weight | 1.8% | "2.8 lb" rounded to a tenth: 0.05/2.8 |
| The sum | 25.7% | rounded up to two figures: **26%** |

- **What it catches:** a factor-level error (a wrong rpm or scale, half the elements). It does not confirm a few percent, and it is not meant to: V1 and V2 carry that. A V3 miss with V1 and V2 inside their bands points to the two gap terms (the stand-in prop, NASA's number), which is why they are listed apart.
- **The record also reports** the line's thrust, the table's 2.995 N and NASA's 3.115 N, with the signed gaps. NASA's rpm precision is not stated to me and is not in the sum.

## V4: D1

Report only: the line's emergent thrust against the chain's k_T w² (the synthetic ramp, `tests/reference_airframe.hpp`), and the wash on the axis at 0.05 m and 0.3 m. The label carries "the load on the blade stand-in `triblade_6inch_scaled`", never "the quad's".

## The records: format and where

- **Format:** Test/Docs' validation record (`01-verification.md`, "Band records"), the blocks `tests/data/airflow/records/d1-hover.md` and `c8-actuator-disc.md` use: `=== validation record: <identity> ===`, `commit:`, `date:`, `path:`, `config:` (Physics' verbatim config line, including the solid counts), then per measure `measure:`, `reference:`, `tolerance:` (the origin names this note's row, "fixed before the run") and `verdict:`. Appended, never edited.
- **Where:** `tests/data/airflow/records/l61-bemt-uiuc.md` for V1 (no grid) and `l61-actuator-line.md` for V2 to V4.
- **Identities** (`TheV1RecordsRead` pins only the C0 and C1 lists, so these need no test edit, but every record must carry a commit, a date and at least one measure, and every banded one a tolerance with its origin):
  - `L61 BEMT, uiuc_apce_10x5 at 5000 rpm, static` and `..., held-out J`;
  - `L61 actuator line, single rotor hover, R/dx 19` (and 25, 38);
  - `L61 actuator line, Phantom 3 at 5300 rpm`;
  - `L61 actuator line, D1`.
- **A record file is read by `AnalysisValidation*`:** it passes before the file merges.

## Not covered

- **A different prop, rpm, grid, thread count or solver change** needs a new measurement, not a wider band.
- **UIUC's own uncertainty** is not quoted (Brandt's thesis is unread); the 2.7% and 1.5% steps stand in for it, and a reading of the thesis would replace them.
- **The 4.4 slope bracket and the V3 sum are judgments,** stated as such. The lead may overrule them; a run does not.
