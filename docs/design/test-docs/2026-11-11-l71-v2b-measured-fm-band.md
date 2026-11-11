# L71 / V2b, the actuator line against UIUC's measured static point: the band, fixed before the first run

**Owner:** Test/Docs. **Status:** written 2026-11-11, before any V2b run. Asked by the lead (L71) after Physics' power-gap diagnosis (`physics/.superpowers/sdd/2026-11-10-wake/power-gap-diagnosis.md`).
**For:** the actuator line on UIUC's APC-E 10x5 geometry (V1's stations, `uiuc_apce_10x5`, the estimated polar as shipped, **uncalibrated**) at UIUC's measured static point: D 0.254 m, 5000 rpm, ρ 1.225. R/dx about 32 (4 mm) on the GPU after P4c, and again after Physics' tip correction; and any finer run Physics proposes.
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. It is not loosened after a run without a lead decision. **The same band applies before and after the tip correction.** That is what makes it the test of the correction.
**Relation to V2:** V2 (`2026-11-11-l61-actuator-line-bands.md`) compares the line with the BEMT. V2b compares it with the measurement. Both stay; V2b does not replace V2's bands.

## Why the band is on the figure of merit

The diagnosis shows the line's power gap against the BEMT is all induced power (the profile power agrees to 1.5%), and that the BEMT with Prandtl itself overstates the tip inflow (§5), so a band against the BEMT cannot say whether the line is right. UIUC's measured static figure of merit for this prop is **0.573** (C_T 0.09636, C_Q 0.005870, C_P = 2π C_Q = 0.03688; FM = C_T^1.5 / (√2 C_P)). The line's FM on the other prop was 0.81 to 0.83; a real small rotor has more induced loss than the ideal.

Absolute C_T and C_Q cannot be banded against the measurement with an uncalibrated polar. Its spread (below) is ±19 to 28% before any other term, and a band that wide tells nothing; V1a already says the same of the static point. **The figure of merit is different:** the lift-side polar freedoms move thrust and torque together and mostly cancel in it (at most 5%); only the drag scale is left. So FM is the one banded quantity, C_T and C_Q are reported.

## The terms (relative changes, as shares of the measured FM)

**1. The reference: 5.6%.** The library's own step between UIUC's static file and its run at J → 0 is +2.7% in C_T and −1.5% in C_P (`2026-11-11-l61-actuator-line-bands.md`). FM goes as C_T^1.5 / C_P, so 1.5 × 2.7 + 1.5 = 5.55%, rounded up. UIUC states no numeric uncertainty (Brandt's thesis is unread), so these steps stand in for it.

**2. The estimated polar's spread: +11% / −16%.** A hover BEMT on the library's `uiuc_apce_10x5` stations with the shipped polar; **only relative changes were printed** (`.agents/refs/l61-sensitivity/new_bemt_fm.py`), never an absolute value and never a comparison with UIUC's data.

| Change to the polar | dC_T | dC_Q | dFM |
|---|---|---|---|
| lift slope 5.5 → 2π / → 4.4 | +10.8% / −15.9% | +13.7% / −18.6% | +2.6% / −5.2% |
| α₀ −3° → −5° / −1° | +14.9% / −14.5% | +19.3% / −17.1% | +3.3% / −4.6% |
| C_d × 0.7 / × 1.5 | +0.3% / −0.4% | −9.0% / +15.1% | +10.3% / −13.7% |

The three freedoms are independent, so the FM term is their root sum of squares: up √(10.3² + 3.3² + 2.6²) = 11.1%, down √(13.7² + 4.6² + 5.2²) = 15.2%, each rounded up. For the same reason the C_T and C_Q spreads are −22% / +19% and −27% / +28%: too wide to band, so they are reported. The brackets (slope 4.4 to 2π, α₀ ±2°, C_d × 0.7 to × 1.5) are my judgments, stated as such. Prandtl's tip loss (dFM −4.4%) is **not a term**: the tip is what V2b tests.

**3. The resolution: 15% × 38 / (R/dx).** V2's rule accepts R/dx 19 when its thrust is within 5% and its power within 7.5% of R/dx 38. With first-order convergence (a judgment: a smeared line kernel is first order) the error left at R/dx 38 is that difference, and at R/dx N it is the difference × 38 / N: 5.0 × 38 / N in thrust and 7.5 × 38 / N in power. In FM, 1.5 × 5.0 + 7.5 = 15% at R/dx 38. A measured pair of V2b runs does not replace this number; replacing it is the lead's decision.

## The band

**FM within [0.573 × (1 − down), 0.573 × (1 + up)]**, where up = 11 + 5.6 + 15 × 38 / N and down = 16 + 5.6 + 15 × 38 / N (percent), N = R/dx as run (4 mm is N = 31.75).

| R/dx N | resolution term | up / down | FM band |
|---|---|---|---|
| 25 | 22.8% | 39.4% / 44.4% | **0.32 to 0.80** |
| 32 | 17.8% | 34.4% / 39.4% | **0.34 to 0.78** |
| 38 | 15.0% | 31.6% / 36.6% | **0.36 to 0.76** |
| 50 | 11.4% | 28.0% / 33.0% | **0.38 to 0.74** |
| 64 | 8.9% | 25.5% / 30.5% | **0.39 to 0.72** |

(Upper edges rounded up, lower edges down, to 0.01.)

- **What a pass means.** It means "not excluded by the measurement and everything we can name", not "right". The band is wide because the polar is estimated; the upper edge is the one that carries information. A line at or above the upper edge is further from the measured world than the terms allow.
- **The direction is reported either way.** The signed gap (line FM − 0.573) is in the record, so a pass at 0.76 and a pass at 0.60 are not read alike.
- **The band was not set from a result.** The diagnosis' FM values (0.81 to 0.83 on the other rotor) were read; the terms above come from the reference, the polar sensitivities and V2's rule. If the 10x5's FM lands within 10% of a band edge it is a finding for the lead, not accepted or refit.

## Report only

- **C_T and C_Q against the measured static values,** as signed gaps in percent, with the polar spreads above beside them.
- **The induced and the profile power** (the diagnosis' split: induced = Σ dT · ua, profile = ω · torque less that), from Physics' probe if it is run at the same point.
- **ua and the angle of attack at r/R 0.875 and 0.975,** the stations the diagnosis names, so the tip correction's effect is read where it acts.
- **Against the BEMT too,** as V2 does, so the two comparisons sit in one record.

## The run

- **The window** follows V2: v_h from momentum theory on the measured thrust (T = C_T ρ n² D⁴ = 3.41 N, v_h = 5.24 m/s, R/v_h = 24.2 ms). The mean is over at least 10 R/v_h (0.24 s), starting at 20 R/v_h (0.48 s) or later. If the last two windows' thrusts differ by more than 2.5%, or their powers by more than 3.75%, the run is unsettled and runs longer; the band is not widened.
- **The line's config** is the same for every V2b run: same stations, same polar, same rpm. A change to the polar or the box is a new band, not this one.
- **The record:** `tests/data/airflow/records/l61-actuator-line.md`, one block per run in the `d1-hover.md` format, identity `L61 actuator line, UIUC 10x5 static, R/dx <N>` (with a line `tip correction: none` or the correction's name), `config:` verbatim from Physics, measures `figure of merit` (banded, tolerance origin "L71 V2b, fixed before the run"), `C_T`, `C_Q`, and the report-only ones. A record `.md` passes `AnalysisValidation*` before it merges.

## Not covered

- **A different prop, rpm or polar** needs its own band. Calibrating the polar (V1a) changes term 2; the lead decides whether V2b is then rerun.
- **The box.** The static rotor's recirculation in a finite box is not a term here. Physics' config line states the box; a different box needs a new statement.
- **The first-order resolution assumption and the polar brackets are judgments.** If the runs at R/dx 32 and 38 do not converge monotonically, the assumption is wrong and the lead is told.
- **UIUC's own uncertainty** is not quoted; the 2.7% and 1.5% steps stand in for it.
