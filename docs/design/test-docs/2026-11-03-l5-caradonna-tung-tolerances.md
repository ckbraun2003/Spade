# L5, Caradonna–Tung blade-resolved: tolerances fixed before the first run

**Owner:** Test/Docs. **Status:** draft for the lead and Physics, 2026-11-03 (the lead's L20). None of it has run. Physics reviews it with L2 to L6.
**For:** L5 of Physics' P5 plan (`../physics/plans/2026-11-03-p5-blade-resolved-plan.md`): one rotor resolved, Caradonna & Tung's model rotor at θ_c 8°, 1250 rpm, M_tip 0.439. P9 deferred these bands to D3 (`2026-11-03-p9-rotor-data-plan.md`, "No tolerance"), and D3 never ran, so they are fixed here.
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. A miss is recorded as a miss.

**The data** are P9's, and nothing here changes them:
- `tests/data/airflow/caradonna1981_rotor.hpp`: Table 17's five stations with their C_L; the two printed C_T, 0.00459 (figure 4) and 0.00460 (figure 6); figure 9's tip vortex path, ±4° and ±0.01;
- its README row;
- the reading's record, `2026-11-03-p9-caradonna-review.md`, with the lead's reading (L7).

The terms below that come from the data were computed from the header's own numbers (Test/Docs, 2026-11-03).

## What can and can't be built term by term

**Built from the data and the measurement:** the tap positions' reading, the integration's spread, figure 9's symbols, compressibility below the true Mach, and the vortex's detection on the grid. Each is computed below.

**Not built term by term: the method's own error at 32 cells across the chord.**
- **The leading edge isn't resolved.** NACA 0012's leading-edge radius is 1.59% of the chord, half a cell. Bouzidi's curvature term there is 3dx²/(8 r_c) ≈ 0.75 dx, as large as halfway's (L2–L3's note, section 1).
- **The boundary layer is thinner than a cell.** At chord Re 1 to 2·10⁶, the turbulent layer at the trailing edge is about 0.37 c Re^(−1/5) ≈ 3.9 mm, under the innermost dx of 5.95 mm. So a wall model can't be matched in its log layer: L3's matching height, 3 cells, is outside the layer.
- **So the method term comes from the literature,** as C0b's 10% does: what an established Navier–Stokes method achieves on this case. Fetch F5 (below) supplies it. The rule is fixed now, and the value is read from F5 before the first run.
- **Taps at x/c < 0.05 are report only,** since the peak sits on an unresolved leading edge.

## The configuration conditions (L6 cites these)

L5's verdicts hold only at this configuration. L6 claims L5's accuracy only where it meets all of them, and its record says which it meets.

| Condition | L5's value | Where it is fixed |
|---|---|---|
| Cells across the chord, innermost level | ≥ 32 (dx 5.95 mm at c 0.1905 m) | The P5 plan |
| The wall rule | Bouzidi, moving, with the moving-wall term and the refill rule Physics names for L2c. Each link's rest state comes off before the sum | L2–L3's note: L2c, and section 2 |
| The wall model | Physics names it: L3's (Spalding, matched 3 cells off), or none. The record states which, and the matching height in y⁺ | L3; open item 1 |
| u_lat at the tip | min(0.254, u_held/1.4) from L1's stability map | L1's note, the decision rule |
| Nest levels | 3 levels, 2:1 by acoustic scaling, the innermost a slab 2.2R × 2.2R × 0.3R about the disc (the plan's cost table); the outer faces as Physics names them for a large still chamber | The P5 plan; L4's note |
| The subgrid model | WALE at C_w 0.5 | L1's note, open item 3 |
| The run | At least 5 revolutions from rest; every measure is the mean over the last revolution and both blades | Fixed here |
| The path | GPU, bit-equal to the CPU reference | The plan's rule |

## The measures and their bands

`u_run` is the run's tip u_lat, and M_run = √3 u_run. At u_run = 0.254 the compressibility terms are the isothermal lattice's against air's adiabatic, which come to at most 4·10⁻⁴ in C_p. Below it, they are Prandtl–Glauert's at each station, `|C_p| (1 − √(1 − M_r²)/√(1 − M_run,r²))` (P9's caveat, as a band term). The table gives them at u_run 0.25, 0.20, 0.15 and 0.10.

### C_p at the five stations

**The measure:** at each station, the largest |C_p − C_p,T17| over the taps with x/c ≥ 0.05, on both surfaces. C_p is read on the surface at the tap's x/c, by the surface-pressure rule Physics names (open item 2). The pressure is normalised by the section's ½ρ(Ωr)², as Table 17's is.

**The band** = m_p (F5) + the tap term + the compressibility term:
- **The tap term:** Table 17 prints x/c to 0.01, so a tap's position is ±0.005 c. Its term is that times the table's own slope between the neighbouring taps, at the station's worst tap.
- **m_p:** the largest disagreement with Table 17 that F5's Navier–Stokes solution shows aft of x/c 0.05, at that station, read from F5's figures (open item 3).

| r/R | Tap term | Compressibility, u_run 0.254 / 0.25 / 0.20 / 0.15 / 0.10 |
|---|---|---|
| 0.50 | 0.0073 | 0.0004 / 0.0005 / 0.0063 / 0.011 / 0.014 |
| 0.68 | 0.030 | 0.0004 / 0.0012 / 0.016 / 0.027 / 0.034 |
| 0.80 | 0.024 | 0.0004 / 0.0017 / 0.023 / 0.038 / 0.048 |
| 0.89 | 0.018 | 0.0004 / 0.0024 / 0.033 / 0.054 / 0.069 |
| 0.96 | 0.036 | 0.0004 / 0.0026 / 0.035 / 0.058 / 0.073 |

The taps at x/c < 0.05 (the leading-edge peak) are printed beside it, report only.

### C_L at the five stations, net of the data's own spread

**The measure, like for like:** our C_p sampled at Table 17's taps, integrated by the same trapezoid rule as `AirflowCaradonnaData.ThePressuresIntegrateNearEachStationsLift`. The reference is Table 17's own pressures integrated the same way, not the printed C_L.
- **Why:** the table's integral sits 0.925 to 1.055 of its printed C_L, because the report's scheme isn't printed. Comparing integral with integral takes that 7.5% out, which is the lead's "net of the spread".

| Measure | Tolerance | Built from |
|---|---|---|
| \|C_L,ours,taps / C_L,T17,taps − 1\| at each station | ≤ m_L + the station's Prandtl–Glauert fraction at u_run | m_L: F5's largest sectional-lift disagreement with the same data, read from its figures. The compressibility fraction is the table's percentage above (0.1 to 8.0%) |
| Our full-surface integral over the printed C_L | report only | It mixes our error with the 7.5% spread |
| Our tap integral over the printed C_L | report only | The same, for the record |

### Figure 9's tip vortex path

**The measure:** the tip vortex's descent z/R and radius r/R at each digitized age. The core is located by the λ₂ minimum in the half-plane at that age behind the blade (open item 4), and the result is the mean over both blades' vortices and the last revolution.

**The band** is the data's ±0.01, plus the age's ±4° times the path's own slope, plus the detection's half a cell. The half cell is taken at the level the core sits in, at most the second: 2 × 5.95 mm / 2R → 0.0052. Below u_run 0.254, a compressibility term adds to the descent: the descent scales with √C_T, so half the C_T term below times z.

| Measure | Points | Tolerance at u_run 0.254 | Built from |
|---|---|---|---|
| \|z/R − z_fig\| | 50°, 162°, 295° | ≤ 0.020 | 0.01 + the slope's 0.0039 (162° to 295°: 0.00098/°) + 0.0052. Below 0.254, add 0.5 × the C_T term × z_fig: 0.0036 at 295° and u_run 0.10 |
| \|r/R − r_fig\| | 49°, 90°, 162°, 293°, 372° | ≤ 0.018 | 0.01 + the slope's 0.0022 (49° to 90°: 0.00056/°) + 0.0052 |

### C_T against the two printed values

**The measure:** the thrust from the blade loads over ρ π R² (ΩR)², the mean over the last revolution. Its distance outside the interval [0.00459, 0.00460] is taken relative to the nearer end.

| Measure | Tolerance | Built from |
|---|---|---|
| \|C_T / C_T,ref − 1\|, outside [0.00459, 0.00460] | ≤ m_T + the C_T compressibility term | m_T: F5's C_T disagreement with the same run, if F5 prints it. The compressibility term, by blade-element momentum with Prandtl–Glauert's lift slope: 0.12% at u_run 0.25, 1.6% at 0.20, 2.7% at 0.15, 3.4% at 0.10, and 0 at 0.254. It is smaller than the sections' because the inflow adapts |
| The chord's caveat: C_T's sensitivity to the chord | report only, 0.59% per 1% of chord | The chord is R/6 from the printed aspect ratio, as the literature reads it. If the aspect ratio counts the span from a root cut-out, the chord is larger by that fraction, which isn't printed. So the effect is printed, not bounded |
| The Reynolds number's caveat: cells across the trailing edge's layer; the wall model's matching height in y⁺ | report only | The layer is under a cell (above). The record shows how far the wall treatment is from resolving it, since the case validates the method, not the skin friction |
| The loads' change between the last two revolutions | report only | Hover's start-up: the record shows how settled the mean is |

If F5 doesn't give a measure (for example, no sectional C_L), that measure's row is report only until a source does.

## The record

`tests/data/airflow/records/l5-caradonna-tung.md` (new). Its opening line is `L5 Caradonna-Tung hover, theta_c 8 deg, 1250 rpm, u_run <u>, <cells> cells per chord, LBM`.
- **`config:`** names every condition in the table above, as run.
- **The measures, in order:** C_p at the five stations, then their leading-edge taps (report only); C_L like for like at the five, then the two report-only integrals; z/R at the three ages and r/R at the five; C_T; then the caveats' lines.
- **`tolerance:`** lines give each band's number with its m from F5, citing `test-docs/2026-11-03-l5-caradonna-tung-tolerances.md, fixed before the first run`.
- It runs opt-in, offline, on the GPU (the plan's "a few revolutions, offline").

## F5, read (L21)

- **The source:** NTRS 19910005815, NASA TM-102862: Srinivasan, G. R., Baeder, J. D., Obayashi, S. & McCroskey, W. J., "Flowfield of a lifting hovering rotor: a Navier–Stokes simulation", European Rotorcraft Forum, September 1990. The title and authors are confirmed from the NTRS record, and they match the note.
- **The files:** the PDF (SHA-256 2495c4d64d4ef2ca61f987cea4a1e98b68b37b55b6e329dbe0c43fb1d7dfa0d9) and NTRS's text (SHA-256 24a625b1877036ed198072133865f73f310f855964b08b5dfea7729dd177a451). Kept in the main checkout's `.agents/refs/srinivasan1990/`, with its PROVENANCE. F5b was not needed.
- **m_p, from fig. 2 (p. 15),** at M_tip 0.44, θ 8°, Re 1.92·10⁶: the five stations. This is Test/Docs' primary reading, pixel-assisted on the figure's own ticks (0.0042 C_p a pixel, ±0.01), and the lead does the second. It is the largest gap between F5's solid (captured-wake) curve and Table 17 aft of x/c 0.05: **0.060** at r/R 0.50, **0.074** at 0.68, **0.077** at 0.80, **0.11** at 0.89 and **0.046** at 0.96.
- **m_L, from fig. 6 (p. 19):** the bound circulation against r/R. Table 17's circulation is (r/R) C_L / 12: 0.0098, 0.0160, 0.0192, 0.0233 and 0.0215. F5's lines cross near the stations, so which one is the M_tip 0.44 fine grid is left to the lead's reading before m_L is set.
- **m_T:** F5 prints no C_T. So, by the rule above, C_T's row is report only until a source gives one.

## Fetches

- **F5.** Srinivasan, G. R., Baeder, J. D., Obayashi, S. & McCroskey, W. J., "Flowfield of a lifting hovering rotor: a Navier–Stokes simulation", NTRS 19910005815 (a NASA conference paper, its PDF listed as available, a US Government work). The authors and year are from memory, to be confirmed when it is read. It is for m_p, m_L and m_T: its Navier–Stokes solution against Caradonna & Tung's Table 17 and C_T at θ_c 8°.
  - If it doesn't plot this case's stations, its NTRS neighbour 19850018358 (Caradonna & Tung's own "Finite-difference computations of rotor loads", NASA TM) is the second choice.
  - Read once, kept in `.agents/refs/`. Figures are read by eye, so the lead does the second reading.

## Open, for Physics before the first run

1. **The wall model at L5:** L3's, or none? At 32 cells per chord the layer is under a cell, so L3's matching height lies outside it. Either way the record says so.
2. **The surface-pressure rule:** how C_p is read at a tap from the fluid nodes near an unresolved surface, for example extrapolated along the normal from two nodes. Name it before the first run. Its error is part of the method term.
3. **m from F5:** the rule is fixed above (F5's largest disagreement at each measure, aft of x/c 0.05). The values are read when F5 is, before the first run.
4. **The vortex's locator:** the λ₂ minimum in the age's half-plane, or another. Name it, and the record prints it.
5. **The outer faces and the box,** for a large still chamber ("special ducting designed to eliminate room recirculation", section 2). Name them, since they set the wake's start-up and the measures' settling.
