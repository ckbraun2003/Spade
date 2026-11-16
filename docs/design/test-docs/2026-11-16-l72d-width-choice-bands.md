# L72d, the actuator line's kernel width: the bands, fixed before any width run

**Owner:** Test/Docs. **Status:** written 2026-11-16, before any of the runs below. Asked by the lead (L72d, rulings L72f). The rule is Physics': `physics/.superpowers/sdd/2026-11-10-wake/line-width-choice-rule.md`.
**For:** the width of the line's kernel, chosen on one measured point and checked on held-out cases. Every run is the line on the GPU, the polar as it stands (estimated, uncalibrated), the tip correction off, the V2b box (`2026-11-11-l71-v2b-measured-fm-band.md`).
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. A band is not loosened after a run without a lead decision. A miss on a held-out case is a finding for the lead, not a retune.
**Relation to V2b:** V2b's figure-of-merit band stands for the uncalibrated line against the measured static point. The tune below is a different job: it picks a number, and its 10% is a selection rule, not a pass band.

## The tune set: how the width is picked

**Case:** UIUC's measured static point of the APC 10x5, 5000 rpm, D 0.254 m, ρ 1.225, R/dx 32 (dx 3.97 mm, V2b's box). Widths tried: 6, 8, 10 and 12 mm (1.5, 2.0, 2.5 and 3.0 cells).

| Quantity | Reference | Source | Within 10% means |
|---|---|---|---|
| C_T = T / (ρ n² D⁴) | **0.09636** | UIUC Propeller Data Site, Vol. 1, `apce_10x5_static_pg0819.txt`, linear in rpm at 5000 rpm (`tests/library_props.gen.hpp`, `uiuc_apce_10x5` ct_static) | 0.08672 to 0.10600 |
| C_Q = Q / (ρ n² D⁵) = C_P / 2π | **0.005870** | the same file and point (cq_static) | 0.005283 to 0.006457 |

I checked the reference against the file itself today (the fetch is below): the rows at 4743 and 5025 rpm give C_T 0.09636 and C_Q 0.005870 at 5000 rpm, so the library's numbers are the file's.

- **The choice:** the smallest width whose C_T **and** C_Q are both within 10%. If none is, the width with the smallest |signed C_Q error|. A tie goes to the smaller width.
- **If C_T leaves ±10% at the chosen width,** that is reported, not tuned away.
- **The width is then fixed** (as mm, or as a share of the chord: the transfer check below says which) and is never chosen again on a held-out case.
- **Where the 10% comes from:** the rule's, approved at L72d. It is wider than the two reference steps between UIUC's static file and its run (C_T +2.7%, C_P −1.5%, `2026-11-11-l61-actuator-line-bands.md`), so the measurement does not move the choice by itself.
- **Not separable:** with an estimated polar, the width absorbs part of the polar's error on one point. The width is a model parameter fitted on one point, not a measurement of the flow. The record says so.

## The held-out bands

None of these is used to choose. Each is fixed now.

| # | Case | Quantity | Reference and source | Tolerance |
|---|---|---|---|---|
| H1 | **V3**, the Phantom 3 stand-in, 5300 rpm, D 9.4 in | the line's thrust per rotor | **3.115 N** (NASA's 12.46 N over four) | **±26%** (2.305 to 3.925 N), unchanged: `2026-11-11-l61-actuator-line-bands.md`, V3 |
| H2 | **V4**, D1's held case, load on the blade stand-in `triblade_6inch_scaled` | the line's emergent thrust per rotor | the chain's **k_T w²** at the held hover (801 rad/s, throttle 0.165), the synthetic ramp of `tests/reference_airframe.hpp`; the run prints it | **±15%** of that value, the signed gap reported |
| H3 | the **same prop at a second rpm** | C_T and C_Q of the 10x5, static, at **3093 rpm** | C_T **0.0937**, C_Q **0.005905** (C_P 0.0371 / 2π) | **±15%** each: C_T 0.07965 to 0.10776, C_Q 0.005019 to 0.006790 |
| H4 | the same prop at a third rpm | C_T and C_Q, static, at **5869 rpm** | C_T **0.0977**, C_Q **0.005936** (C_P 0.0373 / 2π) | **±15%** each: C_T 0.08304 to 0.11235, C_Q 0.005046 to 0.006827 |
| H5 | the **transfer prop** (below) | C_T and C_Q of the UIUC MIT 5x4, static, at **6057 rpm** | C_T **0.18147**, C_Q **0.018924** | **±15%** each: C_T 0.15425 to 0.20870, C_Q 0.01609 to 0.02176; in each of two forms |

**H1.** V3's band is a sum of terms (V2's 10%, V1a's 10%, the stand-in table's 3.9%, NASA's weight 1.8%), it passes for every width in the sweep, and it stays as a guard against a factor-level error. It cannot choose a width, and was never meant to.

**H2.** Physics' number, adopted: 15% is the V2 thrust band (10%) plus 5% for the stand-in blade. It checks that the width does not break the case the website clips run. It is not a precision claim. The reference is a value the case computes, not one I can give here: it goes in the record, with the run line, before the run is read.

**H3 and H4.** The rows are the **nearest** rows of UIUC's static file to 3000 and 6000 rpm, so the runs are at the **actual rpm of the rows, 3093 and 5869**, not at 3000 and 6000 (`SPADE_V2B_RPM=3093` and `5869`). The 15% is the rule's; I read it as the 10% of the tune plus 5% for the rpm. The file moves little with rpm: against 5000 rpm, C_T is −2.8% at 3093 and +1.4% at 5869, and C_P +0.5% and +1.1%. The whole file spans −5.2% to +3.1% in C_T. A width tuned at 5000 rpm therefore carries over to these cases almost by construction. **A pass here is weak evidence,** as the rule says; a miss would be strong. The window follows V2b's (below).

**The fetch.** One file, `https://m-selig.ae.illinois.edu/props/volume-1/data/apce_10x5_static_pg0819.txt`, retrieved **2026-11-16 08:01 local (13:01 UTC)** by `curl` into a fresh, empty folder; read as data with no code run from that folder. 387 bytes, 16 rows (2508 to 6708 rpm). **sha256 `9d4f0434c5e31f6c53082e8551426d2dba40def4a60ee5aca5eeee60a514f2dd`.** The rows used (rpm, C_T, C_P): 3093, 0.0937, 0.0371; 5869, 0.0977, 0.0373; and for the check, 4743, 0.0959, 0.0367 and 5025, 0.0964, 0.0369. UIUC states no numeric uncertainty here; Brandt's thesis stays unread.

## The transfer check: H5

**The prop.** The library's `uiuc_mit_5x4` (UIUC Vol. 2, MIT 5x4, three blades): D **0.127 m**, R 63.5 mm, static point from `mit_5x4_static_0362rd.txt`, linear in rpm at its run's **6057 rpm** (run `mit_5x4_0366rd_6057.txt`, `tests/library_props.gen.hpp`), C_T 0.18147, C_Q 0.018924. Its stations are in the library; nothing is fetched. The static file is not in the repo; the value is the library tool's interpolation of it (retrieved 2026-10-22), as for the 10x5.
**What it tests.** Its diameter is half the 10x5's; its mean chord at 0.75R is 10.62 mm against the 10x5's 16.26 mm (ratio **0.6535**, from the two stations tables). It has three blades, which is a **second variable**: a pass says the width transfers across size and blade count; **a fail says nothing about mm against chord** until a two-blade prop is fetched, which needs the lead's word.

**The kernel's reach.** The width is 1 to 3 cells, so at R/dx 32 (dx 1.984 mm) the widest is 5.95 mm. The 6 to 12 mm of the tune do not all fit on this prop. So the check runs in two forms, as Physics set it:

| Form | Width on the 10x5's chosen W | Run | Limit |
|---|---|---|---|
| **A: the same mm** | W | the largest R/dx the kernel allows, R/dx ≤ 190.5 / W, **plus one coarser R/dx** (about three quarters of it, never below 63.5 / W) as a convergence pair | 6 mm: R/dx 10.6 to 31.8; 8 mm: 7.9 to 23.8; 10 mm: 6.3 to 19.1; 12 mm: 5.3 to 15.9 |
| **B: the same share of the chord at 0.75R** | 0.6535 × W | R/dx 32 | fits only for W up to 9.1 mm (0.6535 × W ≤ 5.95); above that, the largest R/dx the kernel allows, R/dx ≤ 190.5 / (0.6535 × W), and the record says so |

Chord-share widths on this prop: 6 mm gives 3.92 mm, 8 gives 5.23, 10 gives 6.54, 12 gives 7.84. The 10x5's own R/dx 32 is not the mit's: at the same R/dx the mit's dx is half. **If the chosen width does not fit a form at all (below 1 cell), the form is not run and the record says why;** the rule is not changed to make it fit.

**What passes.** A form **passes** when its run (form A: the finer of the pair) has C_T and C_Q both within 15% of H5's reference.
- **Form A convergence flag.** If the pair differ by more than 5% in C_T or 7.5% in C_Q (V2's rule for R/dx 19 against 38), that is reported as a finding. The verdict stands; the band is not widened for resolution.
- **The rule that decides** (the lead, 2026-11-16): the form that passes sets **how the width is chosen**, mm or share of the chord. **If both pass, or neither, it goes to the lead,** not to a default.
- A pass of both forms is not read as "mm, because it is simpler".

## The run window

The same for H3, H4 and H5 as V2b's: the mean over **10 R/v_h starting at 20 R/v_h**, v_h from momentum theory on the measured thrust of that case. The step is D1's, h = 0.38 ms × dx / 4 mm. A run is **unsettled** when the last two windows' thrusts differ by more than 2.5% or their powers by more than 3.75%; it runs longer, and the band is not widened.

| Case | T = C_T ρ n² D⁴ | v_h | R/v_h | Window (starts, ends) |
|---|---|---|---|---|
| 10x5, 5000 rpm (tune) | 3.41 N | 5.24 m/s | 24.2 ms | 0.48 s, 0.73 s |
| 10x5, 3093 rpm (H3) | 1.27 N | 3.20 m/s | 39.7 ms | 0.79 s, 1.19 s |
| 10x5, 5869 rpm (H4) | 4.77 N | 6.20 m/s | 20.5 ms | 0.41 s, 0.62 s |
| mit 5x4, 6057 rpm (H5) | 0.589 N | 4.36 m/s | 14.6 ms | 0.29 s, 0.44 s |

## The records

`tests/data/airflow/records/l61-actuator-line.md`, one block per run in the `d1-hover.md` format. Identity `L72d width, <case>, R/dx <N>, width <w> mm` (and for H5 the form, A or B). `config:` verbatim from Physics, with the width in mm and in cells. Measures `C_T` and `C_Q` with the signed gap in percent against the reference above. The banded ones carry the tolerance origin **"L72d, fixed before the run"**. A record `.md` passes `AnalysisValidation*` before it merges. Where a run is a tune run, the block says "selection, not a band".

## Not covered

- **A different prop, rpm range or polar** needs its own bands. Calibrating the polar changes everything here, and the lead decides whether the checks are rerun.
- **The polar's error is inside the width** (see "Not separable"). A width that passes all of these is a good model parameter on these cases, not a measurement of the flow.
- **UIUC's own uncertainty** is not quoted, for either file.
- **The 15% on H3 to H5 is a judgment** (10% plus 5%, Physics' rule). It is not derived from a spread of the measurement. If any of H3 to H5 lands within 10% of a band edge, that is a finding for the lead, not accepted or retuned.
- **The chord ratio** uses the local chord at r/R 0.75 from the library's stations, interpolated linearly, as the rule says ("at 0.75R"). A mean chord over the blade would give another ratio; the lead sets the definition if it matters.

## Addendum, 2026-11-16 (after the first width runs): H2, and two facts

Written after Physics' results (`l72d-v2b-results.md`, branch physics/line-v2b @ e312e383). Nothing above is edited, and no verdict changes.

**H2: the reference was weak by design.**
- H2 compares the line's thrust on the blade stand-in with k_T w² of the chain's synthetic ramp (`tests/reference_airframe.hpp`, `ct_table` at J 0, C_T 0.12). Physics computed 1.2887 N from the table, not from a running chain. At hover J is near 0, so I expect the same value. I did not check, and a 58% gap does not hinge on it.
- The ramp is a placeholder. No measurement stands behind it, and it is not the stand-in's blade. H2's "plus 5% for the stand-in blade" assumed the two would agree. That assumption was mine, and it was wrong: the design note (p4, section 3) already says they differ by construction.
- **Ruling.** H2 as written is a FAIL (line 2.038 N against 1.289 N, +58.2%), and it stays recorded so. It is not re-banded. It is withdrawn as evidence for or against the width, because it measures the blade against the ramp. The width rests on the tune, H1, H3, H4 and H5.
- **No switch to the BEMT as the reference.** The line gives 2.038 N and the BEMT 2.03 N. Choosing the BEMT after seeing the result would make H2 pass by selection. The BEMT also runs on the same estimated polar, so it is not independent of the line. The agreement can be reported as an unbanded consistency row: the line against the BEMT, about +0.4%, labelled "set after the result".
- A real V4 band needs a reference measured for a prop of that size, blades included. The repo holds none. Whether to get one is the lead's call, and it would need a fetch word.

**Two facts for the record.**
1. **The 12 mm run is at R/dx 31, not 32.** The tune table above gives 12 mm as 3.0 cells. At R/dx 32 it is 3.02 cells, over the kernel's 3. Physics ran R/dx 31 (2.93 cells), a 3% change in dx. The bands are unchanged, and the 12 mm row reads R/dx 31.
2. **The pick sits on the cap.** C_T and C_Q improve at every step to wider kernels, and the widest allowed width wins. The rule picked from the widths tried, and it did not find an optimum. Read with "Not separable" above: the width absorbs the polar's error, and C_Q stays low in every case (-11% to -30%). The record says so. H4's C_Q passes by 0.04% of its band, which the note makes a finding for the lead, neither accepted nor retuned.
