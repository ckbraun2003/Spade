# L1, the LBM lattice: tolerances fixed before the first run

**Owner:** Test/Docs. **Status:** reviewed by Physics, 2026-11-03 (OK, its answers folded in below). None of these cases has run.
**For:** L1 of Physics' P5 plan (`../physics/plans/2026-11-03-p5-blade-resolved-plan.md`, "The plan (the LBM route)"), after Cameron's D9 and D10 (backlog, 2026-11-03). As plan A's Task 11 did for the NS tier, this note fixes each case's configuration, measures, tolerances and record before the first run. L1's tests read their tolerances from here, and each record's `tolerance:` line cites this note.
**The rule** (`01-verification.md`, "Validation against published data"): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. It is not loosened after a run without a lead decision recorded with its reason. A miss is recorded as a miss.

Everything is in lattice units (dx = dt = 1). The sound speed is c_s = 1/√3, so the lattice Mach is Ma = √3 u_lat, and ν_lat = (τ − ½)/3, with τ = 3(ν + ν_t) + ½ under WALE (spec §11.3).

| u_lat | 0.05 | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | 0.35 | 0.40 |
|---|---|---|---|---|---|---|---|---|
| Ma | 0.087 | 0.173 | 0.260 | 0.346 | 0.433 | 0.520 | 0.606 | 0.693 |
| Ma² | 0.0075 | 0.030 | 0.068 | 0.120 | 0.188 | 0.270 | 0.368 | 0.480 |

## What differs from the NS tier's bands, and why

**1. The Mach term.** The NS tier projects to ∇·u = 0 every step, so against an incompressible reference its error is the discretization's alone. The LBM is weakly compressible: at lattice Mach Ma it carries an O(Ma²) error against Ghia and the DNS, both incompressible. The error has two parts:
- The density follows the pressure, δρ/ρ = Ma² p′/(ρU²), so the flow is not quite divergence-free.
- D3Q27 cannot hold the diagonal third moment (`xxx`: on its velocities c_x³ = c_x), so the normal stresses keep an O(Ma²) error. The third-order regularization corrects the off-diagonal ones (spec §11.3's references).

The NS bands have no such term, and it does not shrink with dx at a fixed u_lat. So:
- Each band below carries a Mach allowance at the case's u_lat, from the case's own pressure range or energy.
- C0a measures the term on its own, as a series in u_lat at a fixed grid and τ.
- C1 runs at u_lat 0.05 as well as 0.1, against the same band. A miss at 0.1 alone then shows the Mach term.
- Order checks run under diffusive scaling (u_lat ∝ dx, τ kept), where the discretization and Ma² both fall as dx². At a fixed u_lat, Ma² is a floor.

At Caradonna–Tung's own tip Mach (L5), the lattice's compressibility is the flow's, not an error. Only the equation of state differs: the lattice is isothermal, and at M 0.44 its density differs from air's adiabatic by about 0.2%.

**2. The acoustic time step.** The NS tier's step is advective (U h/dx = 0.5 in C1). The LBM's is the lattice's, U dt/dx = u_lat = 0.1, so the same T and dx take five times the steps. That changes three things:
- **No splitting term.** The NS bands carried MacCormack's first-order splitting error (the solids note's S1). The LBM is second order in space and time together, so its time error is inside the (k dx)² term.
- **The flow carries sound, and only viscosity damps it.** An impulsive lid, or a density not matched to the pressure, launches sound. The NS tier has none. A box's slowest acoustic mode decays as e^(−ν k² t) at least, with the bulk viscosity 2ν/3 of a single-τ collision (WALE and the walls only add to the damping). In a cavity that leaves 1% after T = 0.47 Re in units of L/U, so C1's T grows with Re, and C1's record gains a steadiness line. In C0's 2π box the slowest sound has a period of about 1.1 t* at u_lat 0.1, and its exchange with K would show in −dK/dt. So C0a and C0b start from the density matched to the analytic pressure, ρ = ρ0 + p′/c_s².
- **u_lat is configuration.** Every record names u_lat, Ma, ν_lat and τ, the time in the reference's units, and its steps.

**3. The subgrid model is on.** The NS tier's C1 ran without one. L1 runs WALE in τ in C0b, C1 and the map, at C_w = 0.5, named by Physics before any run (open item 3). In two dimensions WALE's operator is ⅔(det ∇u)². That is zero in pure shear, but not in a vortex core, and WALE is non-zero in solid rotation. So spec §11.3's "no eddy viscosity in laminar flow" overstates: in the cavity WALE's viscosity is small, not zero.
- **The estimate.** In a core of vorticity ω, ν_t = (C_w Δ)² |ω| (1/24)^¼. Ghia et al.'s table of vortex centres gives core vorticities of about 3.2 and 2.3 U/L. At C_w 0.5 and 128², that is 0.2% of ν at Re 100 and 0.6% at Re 400.
- **The records.** C1's record reports the peak ν_t/ν. C0a runs with WALE off, since its exact solution has no subgrid term.

**Unchanged:**
- Ghia's file, its 17 points a centreline, and Table II's point 117 left out of v at Re 400 (decided 2026-10-12, before any run).
- The spec's tolerances (§11.2): 0.02 at 128² for Re 100 and 400. At 128³ with the subgrid model, the dissipation peak within 10% and its time within 5%.
- The record's shape (`01-verification.md`, "Validation records").
- A miss is a miss.

L1's GPU twin is bit-equal to its CPU reference (the plan's rule), so one run's verdict holds for both, and the `path:` line says which ran.

## C0a. The Taylor–Green mode's exact decay

**Reference.** The two-dimensional mode in a periodic box of side 2πL:
- `u = U0 sin(x/L) cos(y/L)`, `v = −U0 cos(x/L) sin(y/L)`, `w = 0`;
- `p′ = (ρ0 U0²/4)(cos(2x/L) + cos(2y/L))`.

It solves the incompressible Navier–Stokes equations at any Re (Taylor 1923, Phil. Mag. 46). Its nonlinear term is a gradient, so `K(t) = K(0) e^(−4νt/L²)` exactly.

**What it checks.** The lattice's own viscosity and the Mach term, with no wall, no subgrid model and no turbulence. A wrong c_s² or τ gives ν off by a factor, far outside the band.

**Configuration.**
- **The box:** an N × N × 1 slab, periodic.
- **The collision:** WALE off (C_w = 0), ν_lat 0.01 (τ 0.53, near the tier's ½).
- **The start:** ρ from the analytic pressure. The config line says how the populations start.
- **When it is read:** at `n = L²/(4ν_lat)` steps, rounded to the nearest step, where K has fallen to about e⁻¹. That is 2594 steps at N 64 and 648 at N 32.

**Measure.** The effective viscosity from the energy's decay is `ν_eff = −L² ln(K(n)/K(0)) / (4n)`, where K is the mean of ½|u|² over the cells, summed in double. The banded measure is its size, `|ν_eff/ν − 1|`. The series below prints it signed.

| | N 32, u_lat 0.1 | N 64, u_lat 0.05 | Built from |
|---|---|---|---|
| Tolerance on \|ν_eff/ν − 1\| | ≤ 0.11 | ≤ 0.027 | The lattice is second order: at most (\|k\| dx)² = 2(2π/N)², which is 0.077 and 0.019. The coefficient is taken as 1, which is generous (a central difference's is 1/12); no published coefficient for this collision is relied on. The Mach term is at most Ma², 0.030 and 0.0075; the stored energy of the sound alone is Ma²/8 of K |
| Order check | | e(64) ≤ 0.5 e(32) | Diffusive scaling: N doubles, u_lat halves and τ is kept, so both terms fall by 4. The check asks an observed order of at least 1, half the method's. It is reported, not checked, when e(32) is under a tenth of its band |

**The Mach series (report only).**
- **The runs:** N 64 and ν_lat 0.01 as above, at u_lat 0.1, 0.15, 0.2, 0.25 and 0.3. τ is the same in each, so only Ma changes.
- **What each prints:** `ν_eff/ν − 1`, and beside it the term's coefficient. The coefficient is the difference from the u_lat 0.05 run over the difference in Ma².
- **What it is for:** the accuracy half of the u_lat decision for Caradonna–Tung (the map is the stability half), and the measured check on the Mach allowances in this note.
- **If the coefficient comes out above 1,** the allowances here are wrong. Report it to the lead; the bands stand until the lead decides.

## C0b. The Taylor–Green vortex at Re 1600, against DNS

**Reference.** Spec §11.2's: the dissipation rate ε*(t*) of a spectral DNS at Re 1600 (van Rees et al. 2011, J. Comput. Phys. 230, beside Brachet et al. 1983). Its peak is about 0.0127, near t* ≈ 9. Here t* = t U0/L, K* = K/U0², and ε* = −dK*/dt*.

**The data:** `tests/data/airflow/tgv1600_spectral.hpp` (the lead's fetch L14, 2026-11-03), with its provenance in the header and `tests/data/airflow/README.md`.
- **What it is:** the High-Order CFD Workshop's case C3.5 reference, tabulated: E_k and ε at t* 0 to 19.99 by 0.01. It is UCL's dealiased pseudo-spectral solution, grid-converged on 512³. The case cites van Rees et al. (2011) for the grid-convergence study, so spec §11.2's "DNS data of van Rees et al." means this file.
- **Its peak:** ε* 0.012857528187 at t* 8.97.
- **How it got here:** transcribed by machine, with no figure read and no reading error to carry.
- **The horizon:** the file ends at t* 19.99, so C0b's series runs to t* 19 and its run to t* 20.

**Configuration.**
- **The field:** the three-dimensional field in a periodic box of side 2πL.
  - `u = U0 sin(x/L) cos(y/L) cos(z/L)`, `v = −U0 cos(x/L) sin(y/L) cos(z/L)`, `w = 0`;
  - `p′ = (ρ0 U0²/16)(cos(2x/L) + cos(2y/L))(cos(2z/L) + 2)`, with ρ from p′.
- **The flow:** u_lat 0.1 (U0; Ma 0.173) and Re = U0 L/ν = 1600.
- **The collision:** WALE on, at C_w 0.5.
- **The horizon:** t* = 20.

| N | ν_lat (τ) | Steps to t* 20 |
|---|---|---|
| 64³ | 6.37e-4 (0.50191) | 2037 |
| 128³ | 1.27e-3 (0.50382) | 4074 |

**Measures.**
- **K*:** sampled every Δt* of at most 0.05 (every 5 steps at 64³, every 10 at 128³), summed in double.
- **ε*:** central differences of those samples. The peak is the largest sample, unsmoothed, with its t*.
- **The series:** the record prints ε* at t* = 1, 2, …, 19, report only, each beside the DNS's value at that t*.

| Measure | 64³ | 128³ | Built from |
|---|---|---|---|
| The peak ε* | report only | within 10% of the DNS's | Spec §11.2, the literature's LES band at 128³. The Mach term is at most Ma² = 3% of the peak (C0a's coefficient bound), under a third of the band |
| Its t* | report only | within 5% of the DNS's | Spec §11.2. The sampling's reading error, Δt*/2 = 0.025, is 0.3% of t* 9. The Mach term's effect on the time is not estimated |

## C1. The cavity against Ghia

**Reference.** Ghia, Ghia & Shin (1982), Tables I and II (`tests/data/airflow/ghia1982_cavity.hpp`), unchanged.

**Configuration.**
- **The box:** a 128 × 128 × 1 slab, z periodic.
- **The walls:** the four side faces are walls by halfway bounce-back. The lid at y = 1 moves at u_lat by the moving-wall term. L1 gains these plain box-face walls for C1; L2 keeps the part walls (open item 1).
- **The start:** impulsive, from rest at ρ0.
- **The collision:** WALE on.
- **Units:** L = 128 cells, so the time unit L/U is 128/u_lat steps.

| Re | u_lat | ν_lat (τ) | T (L/U), steps |
|---|---|---|---|
| 100 | 0.1 | 0.128 (0.884) | 50, 64 000 |
| 100 | 0.05 | 0.064 (0.692) | 50, 128 000 |
| 400 | 0.1 | 0.032 (0.596) | 190, 243 200 |
| 400 | 0.05 | 0.016 (0.548) | 190, 486 400 |

T is 0.47 Re rounded up, which leaves the slowest sound at 1% (above). The NS tier's T were 40 and 60.

**Sampling.** The nodes sit at (i + ½)/128, so x = 0.5 falls between two node columns, and u on it is their mean.
- **Along y:** u is linear between nodes, and from the last node to the wall's velocity at the wall (0 at the floor, U at the lid), as in the NS test.
- **v on y = 0.5:** the same way, from the two node rows, and to 0 at the side walls.

**Measures and tolerances,** the same for all four rows:

| Measure | Tolerance | Built from |
|---|---|---|
| max \|u − u_Ghia\| over Table I's 17 points | ≤ 0.02 | Spec §11.2, unchanged. The LBM adds two terms. The Mach term is Ma² times the interior pressure's range, taken as 0.25 ρU² away from the lid's corners: 0.008 at u_lat 0.1 and 0.002 at 0.05. WALE adds at most 0.002 (above). At u_lat 0.05 they leave the band nearly whole; at 0.1 they take up to half of it. So a miss at 0.1 alone, with 0.05 within, is the Mach term, and says what the tier's default u_lat costs at this size |
| max \|v − v_Ghia\| over Table II's 17 points (16 at Re 400, point 117 left out) | ≤ 0.02 | The same |
| Steadiness: the largest change of any of the 34 sampled values over the last 1.0 L/U, against its value at T, sampled every 0.05 L/U | ≤ 0.002 | A tenth of the band. The slowest sound's period is 0.35 L/U at u_lat 0.1 (0.17 at 0.05), so the window holds several. A miss means T did not reach the steady state. The record stays, and a longer T is a new configuration, fixed before its run |
| The peak ν_t/ν over the cells at T | report only | WALE in a laminar vortex (above) |

## The stability map

**What it decides:** u_lat for Caradonna–Tung (L5). Its true tip Mach, 0.439, is u_lat 0.254 (the plan, "What the measurements do not settle").

**The range.** The plan's "lattice Mach (0.1 to 0.3)" is u_lat 0.1 to 0.3, which is Ma 0.17 to 0.52 (Physics, open item 2). Read as Ma, it would stop at u_lat 0.17, below Caradonna–Tung's 0.254, and decide nothing. The map runs on to u_lat 0.40 for the margin (below).

**The flow:** C0b's field at 64³, with WALE on at C_w 0.5 and ρ from p′, run to t* = 20 (`203.7/u_lat` steps).
- **What it tests:** the field's transition carries the grid to its smallest scales, which is where an LBM goes unstable.
- **What it cannot test:** after that, a decaying flow tests nothing. So the map covers the lattice's bulk through transition only. Walls (L2), nests (L4), and a sustained flow over many revolutions (L5) each carry their own stability in their own records.

**The points.** u_lat 0.10, 0.15, 0.20, 0.25, 0.30, 0.35 and 0.40, each at ν_lat 0 (τ = ½, WALE's viscosity alone), 1e-6, 1e-5, 1e-4 and 1e-3, which makes 35 points. Four required rows follow (below), so the map is 39 points.
- **Why these ν_lat:** Caradonna–Tung's levels sit between about 4e-7 and 5e-6. Its finest level, at dx 5.95 mm, has ν_lat = ν u_lat/(U_tip dx) = 1.7e-5 u_lat for air's 1.5e-5 m²/s, and acoustic scaling halves that on each coarser level. So ν_lat 0 and 1e-6 bracket its coarse levels, and 1e-5 its finest.
- **Required rows: the cases' own points** (Physics' review of the L2–L6 bands, A2, 2026-11-04; the lead's L26). Each is a point the later cases run at, and the collision operator the map picks must hold at all four with WALE on:
  - L2a: u_lat 0.025 at ν_lat 1.5e-4 and 3e-4 (τ − ½ = 4.5e-4 and 9e-4);
  - L4d: u_lat 0.05 at ν_lat 9.4e-6 (τ − ½ = 2.8e-5);
  - L5: u_lat 0.254 at ν_lat about 4.2e-6 (τ − ½ = 1.3e-5, from Re_c 1.95·10⁶ and 32 cells to the chord).
  These are far past what plain BGK holds. The map runs them with WALE on as it runs the others. L2a itself runs with WALE off (L2–L3's note), and WALE's viscosity only adds, so a row that holds with it bounds a case that runs without it.
- **How the record prints them:** ν_lat and τ − ½ as the solver holds them in fp32. One ulp of τ near ½ is 6e-8, so at ν_lat 1e-6, 3ν_lat is only 50 ulps.

**Held, fixed now:** K* finite and at most 1.05 K*(0) at every sample (every 5 steps) to the horizon.
- An unstable LBM's energy grows without bound, so any fixed ceiling catches it before the first non-finite value.
- The 1.05 leaves room for K's exchange with the pressure field. For this field, p′²/(2ρc_s²) over K is 0.07 Ma², under 4% of K at u_lat 0.4.

**The decision, fixed now.** The margin is 1.4, Physics' measure from P9 (open item 6).
- **Why 1.4:** Table 17's largest −C_p (`caradonna1981_rotor.hpp`) is 1.18 at r/R 0.89 and 1.12 at 0.96, both on the upper surface near the leading edge. The local speed is √(1 − C_p) times the section's, 1.32 and 1.40 V_tip. An isentropic reading of the same C_p gives 1.33 and 1.42 V_tip (a local Mach of 0.635 at 0.96). Either way the needed u_lat, 0.254 × 1.4 = 0.355 (0.360 isentropic), lies between the map's points 0.35 and 0.40, so the decision is the same.
- **The rule:** let u_held be the largest u_lat at which every ν_lat point held. Caradonna–Tung runs at the smaller of 0.254 and u_held / 1.4. So it runs at its true tip Mach only if every point held at u_lat 0.40.
- **Below 0.254:** if u_held / 1.4 is under 0.254, P9's compressibility caveat applies, as the plan says.
- **If no u_lat holds,** the lead decides.

## The records

**Where they go.** Each case's blocks go in its file, in the shape of `01-verification.md`, "Validation records" (`AnalysisValidation*` reads them). Each opening line ends with `LBM`, so no LBM record takes an NS record's identity.

**What the lines hold:**
- **`config:`** names:
  - N and the box;
  - the lattice and the collision (`D3Q27, recursive-regularized to 3rd order`);
  - WALE and its C_w, or `WALE off`;
  - u_lat and Ma, ν_lat and τ;
  - T in the case's units, with its steps;
  - how ρ and the populations start;
  - in C1, the walls.
- **`path:`** is the CPU reference (toolchain, preset) or the device's `describe()` line.
- **`tolerance:`** lines cite `test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run`.

| Record | File | Opening line | Measures, in order |
|---|---|---|---|
| C0a | `c0-taylor-green.md` | `C0 Taylor-Green, the 2D mode's exact decay, 64x64x1, u_lat 0.05, LBM` (one per N and per series point) | `\|nu_eff / nu - 1\|` with its tolerance. At N 64, the order check `e(64) / e(32)`, tolerance `<= 0.5`, or report only when e(32) is under a tenth of its band. In the series, `nu_eff / nu - 1` signed and the coefficient, both report only |
| C0b | `c0-taylor-green.md` | `C0 Taylor-Green vortex, Re 1600, 128^3, LBM` (and 64^3) | The peak `epsilon*`; its `t*`; then `epsilon*` at `t*` 1 to 19, report only, each `reference:` line giving the DNS's value |
| C1 | `c1-cavity.md` | `C1 lid-driven cavity, Re 100, 128x128, u_lat 0.1, LBM` (four, one per row) | max `\|u - u_Ghia\|`; max `\|v - v_Ghia\|`; the steadiness; the peak `nu_t / nu`, report only |
| The map | `l1-stability-map.md` (new) | `L1 stability map, 64^3 Taylor-Green, u_lat 0.25 (Ma 0.433), LBM` (one per u_lat: the seven above and 0.025, 0.05 and 0.254 for the required rows) | For each ν_lat in order: the steps held, of the horizon; K* over K*(0) at the last sample held. Both report only |

**Two parser rules shape the lines** (`engine/analysis/validation.cpp`):
- A measure's value must start with a number, with a unit after it if any. Readers split at the last ` = `.
- A tolerance is `<bound> (<where it was fixed>)`, and its bound holds no ` (`. A report-only measure carries `tolerance: none (report only)`.

A C1 block and a map block (illustrative values):

```text
=== validation record: C1 lid-driven cavity, Re 400, 128x128, u_lat 0.1, LBM ===
commit: 0123abc (physics/lbm-l1)
date: 2026-11-10
path: CPU reference, MSVC 1943, release
config: 128x128x1 slab (z periodic), D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.1 (Ma 0.173), nu_lat 0.032 (tau 0.596), Re 400, T 190 (243200 steps), walls by halfway bounce-back, the lid by the moving-wall term, impulsive start at rho 1, fp32
measure: max |u - u_Ghia| on x = 0.5 = 0.0123
reference: Ghia, Ghia & Shin (1982), Table I (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: max |v - v_Ghia| on y = 0.5, Table II point 117 left out (printed -0.23827, a suspected typo in the original) = 0.0123
reference: Ghia, Ghia & Shin (1982), Table II (tests/data/airflow/ghia1982_cavity.hpp)
tolerance: <= 0.02 (spec §11.2; test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the largest change of the 34 sampled values over the last 1.0 L/U = 0.0001
reference: a steady state (the cavity's sound damped)
tolerance: <= 0.002 (test-docs/2026-11-03-l1-lbm-tolerances.md, fixed before the first run)
verdict: within
measure: the peak nu_t / nu over the cells at T = 0.001
reference: none (WALE in a laminar vortex)
tolerance: none (report only)
verdict: report only
```

```text
=== validation record: L1 stability map, 64^3 Taylor-Green, u_lat 0.25 (Ma 0.433), LBM ===
commit: 0123abc (physics/lbm-l1)
date: 2026-11-10
path: CPU reference, MSVC 1943, release
config: 64^3 periodic 2 pi box, C0b's field, D3Q27 recursive-regularized to 3rd order, WALE C_w 0.5, u_lat 0.25 (Ma 0.433), to t* 20 (815 steps), rho from the analytic pressure, fp32; held: K* finite and <= 1.05 K*(0) at every 5th step
measure: steps held at nu_lat 0 (tau - 1/2 = 0), of 815 = 815
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
measure: K* / K*(0) at the last sample held, nu_lat 0 = 0.123
reference: none (the lattice's own stability)
tolerance: none (report only)
verdict: report only
```

The map's block continues with the same two measures at ν_lat 1e-6, 1e-5, 1e-4 and 1e-3.

## Where they run

- **Opt-in:** every case, with `SPADE_AIRFLOW_VALIDATION=1`, in release through `--gtest_filter`, as plan A's were.
- **The gate:** one case may join it: C0a at N 32 (648 steps of 1024 cells), a regression guard like the NS tier's 32² cavity. It joins only if it times under 30 s in debug, under ctest at `-j 1` (`02-build-and-gate.md`; time it as gap 5 did).
- **Not the gate:** C1's rows (64 000 steps at the least) and C0b.

## Open, for Physics before the first run

All six are answered (Physics' review, 2026-11-03).

1. **C1's walls.** Closed: L1 gains plain box-face walls (halfway bounce-back on the domain faces, the lid by the moving-wall term), so C1 stays in L1. L2 keeps the part walls (links from the SDF, Bouzidi, moving parts). Physics corrects L1's face list in its plan.
2. **The map's range.** Closed: u_lat 0.1 to 0.3 (Ma 0.17 to 0.52). Physics fixes the plan's wording. The map adds 0.35 and 0.40 for item 6.
3. **C_w.** Closed: C_w = 0.5, Nicoud & Ducros (1999)'s own value (OpenFOAM's 0.325 is a later recalibration), named before any run.
   - C0 and C4 confirm it, so neither picks it and neither verdict is circular.
   - A change after a run is a named cause (`TD-2`) and a new configuration, and the old record is kept.
4. **WALE in laminar vortices.** Closed: its S^d vanishes in pure shear, not in rotation. Physics corrects spec §11.3's line on its L1 branch. C0a runs with WALE off, and C1 reports ν_t/ν.
5. **C0b's DNS data.** Closed: in the repo (`tgv1600_spectral.hpp`, fetch L14), its provenance in the header.
6. **Caradonna–Tung's margin.** Closed: 1.4, not 1.2, measured from P9's Table 17 (the stability map, above).
