# L2 and L3, the LBM's walls and wall model: tolerances fixed before the first run

**Owner:** Test/Docs. **Status:** draft for the lead and Physics, 2026-11-03. None of these cases has run. Physics reviews it before L2's GREEN.
**For:** L2 (walls) and L3 (the wall model) of Physics' P5 plan (`../physics/plans/2026-11-03-p5-blade-resolved-plan.md`, "The plan (the LBM route)"), in the shape of L1's note (`2026-11-03-l1-lbm-tolerances.md`). This note fixes each case's configuration, measures, tolerances and record before the first run. L2's and L3's tests read their tolerances from here, and each record's `tolerance:` line cites this note.
**The rule** (`01-verification.md`, "Validation against published data"): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. It is not loosened after a run without a lead decision recorded with its reason. A miss is recorded as a miss.

Everything is in lattice units (dx = dt = 1), as in L1's note: c_s = 1/√3, Ma = √3 u_lat, ν_lat = (τ − ½)/3.

## How the bands are built

As the solids note's were (`2026-10-13-solids-tolerances.md`): every reference here is exact, or a closed form. So a band is the method's own error at the stated resolution, estimated term by term, each a worst case, summed, and rounded up to two figures. A correct solver passes with room. A link in the wrong direction, a wall a cell off, a sign error or a lost moving-wall term does not.

## What differs from L1's bands and the NS tier's, and why

**1. Where the wall sits.** The two bounce-backs place the wall to different orders.
- **Halfway bounce-back** puts the wall halfway along each cut link, so it is within dx/2 of the true surface. That is first order, as the NS tier's aperture ramp is. Its term is the measure's sensitivity to a wall moved dx/2, the solids note's method.
- **Bouzidi's interpolated bounce-back** (Bouzidi, Firdaouss & Lallemand 2001, Phys. Fluids 13) takes the wall's fraction q along each link from the SDF, as the linear root φ(x_f)/(φ(x_f) − φ(x_s)). On a surface whose smallest radius of curvature is r_c, that root is off by at most (|c| dx)²/(8 r_c) ≤ 3 dx²/(8 r_c), with |c| ≤ √3. That is second order, and its term is the sensitivity to a wall moved that far.

**2. The loads in fp32.** Each cut link's momentum exchange carries about 2 w_i ρ0 c_i of the rest state, the ambient pressure ρ0 c_s².
- **In exact arithmetic it cancels.** Along every lattice line, each entry into the solid is matched by an exit, in the opposite direction with the same weight. So the rest state's sum over a closed body is zero, line by line.
- **In fp32 it does not.** The sum's rounding scales with the rest state's partial sums, not with the load. The rest state over the dynamic pressure is c_s²/(½u²) = 2/(3u_lat²): 1070 at L2a's u_lat 0.025, and 67 at u_lat 0.1. A random-walk estimate puts L2a's side forces at about 0.01 of ½ρU²πR², and the plate's couple at about a quarter of itself.
- **So every band below assumes the rest state comes off each link before the sum.** Either f is stored as f − w_i ρ0, or each link's exchange subtracts 2 w_i ρ0 c_i. Both are exact, since the rest state cancels line by line. L5's blade loads need the same (open item 1).

**3. Moving walls.** A moving wall adds the moving-wall term to each link, and a node the wall uncovers is refilled.
- **The refill's error is bounded by its gross rate.** For the rotating plate (L2c), the angular momentum carried by the nodes uncovered each step is 0.035 Re of the couple (by quadrature over the spheroid's trailing surface, at ρ0 and the wall's velocity). A refill rule that got every node wholly wrong would move the mean couple by at most that.
- **S2's Couette (L2b) turns a curved part without changing its links**, since a cylinder looks the same at every angle. So it checks the moving-wall term with no refill, and the plate then adds the refill alone.
- **The momentum exchange's moving-wall form,** for example Wen et al.'s Galilean-invariant one (2014, J. Comput. Phys. 266), and the refill rule are Physics' to name. Each record's `config:` line names both (open item 4).

**4. Sound from the start.** A wall that appears in a moving fluid, or a field that doesn't match the wall, launches sound of pressure ρ0 c_s u_n, where u_n is the mismatched normal velocity. In C_p that is 2 u_n/(Ma U): at Ma 0.043, a staircase's normal flux of a tenth of U gives a pulse of C_p ≈ 5. The NS tier's projection removes it in one step. The LBM radiates it at c_s, so:
- L2a starts from the matched potential flow, not from a uniform stream.
- L2a is read only after the near field has settled, and before the faces' reflections return.

**5. The Mach term** is as in L1's note: O(Ma²) against an incompressible reference. Each case runs at a u_lat small enough that the term is a small, stated part of its band.

**Unchanged:**
- The rule, the record's shape (`01-verification.md`, "Validation records"), and a miss is a miss.
- The GPU twin is bit-equal to the CPU reference (the plan's rule), so one run's verdict holds for both, and the `path:` line says which ran.

## L2a. S1's sphere, in the LBM

**Reference.** The NS tier's S1 reference: potential flow past a sphere (Batchelor 1967, ch. 6), evaluated in double at each sample point. With a = (R/r)³: `u_r = U cos ϑ (1 − a)`, `u_ϑ = −U sin ϑ (1 + a/2)`, and `C_p = 1 − |u|²/U²`.

**The plan's "against the NS tier's record".** Both tiers compare against the same reference, each with its own band. The NS tier's own number (0.0813 at R/dx 6, `s1-sphere.md`, f4ddf75) is our solver's output, not a reference, and it was sampled a cell nearer the wall. So this note quotes it for the tiers' comparison and checks nothing against it.

**Configuration.**
- **The box:** a cube of side 16R, the sphere centred midway between nodes on all three axes, so its links are mirror-symmetric in y and z. Inflow U on −x by equilibrium; the other faces zero-gradient. R/dx 6 (96³) and 12 (192³).
- **The flow:** u_lat 0.025 (Ma 0.043), Re = 2UR/ν = 2000: ν_lat 1.5e-4 (τ 0.50045) at R/dx 6 and 3.0e-4 (τ 0.5009) at 12. WALE off: the reference has no subgrid term, and WALE is non-zero in irrotational strain.
- **The start:** u from potential flow, ρ = ρ0 + p′/c_s² with Bernoulli's `p′ = ½ρ0(U² − |u|²)`, and the populations at equilibrium.
- **The walls:** halfway, then Bouzidi; each runs at both sizes.

**When it is read.** At n₁ = ⌈4√3 R⌉ steps (four acoustic times R/c_s): 42 and 84 steps, or tU/R = 4 Ma = 0.173.
- **After the start's sound:** the near field settles within a few R/c_s. A second reading at n₂ = ⌈5√3 R⌉ (52 and 104 steps) prints the change, report only.
- **Before the reflections:** the faces are 7.5R from the surface, so sound from the start returns at 15R/c_s.
- **Before separation:** an impulsively started cylinder's boundary layer first separates at tU/R ≈ 0.35. The sphere's slip velocity is three quarters of the cylinder's, so its first separation comes later.
- **The boundary layer is thin then:** its displacement thickness is δ* = 2√(νt/π) = 0.0148 R at n₁ (Rayleigh's layer), 0.089 and 0.18 cells.

**The samples.** Two cells out along the outward normal, every 15° in ϑ on four meridians (52 points), trilinear in ρ, with p′ = c_s²(ρ − ρ0). Each is compared with potential flow at the same point.
- **Why two cells:** at two cells out, the trilinear stencil's eight nodes lie at least 2 − √3 = 0.27 dx outside the wall, so all are fluid.
- **The NS tier sampled one cell out.** In the LBM a stencil there can reach a solid node, which has no density.

| Measure | Halfway, R/dx 6 | Halfway, 12 | Bouzidi, 6 | Bouzidi, 12 |
|---|---|---|---|---|
| max \|C_p − C_p,pot\| over the 52 samples, at n₁ | ≤ 0.22 | ≤ 0.17 | ≤ 0.11 | ≤ 0.066 |
| max(\|F_y\|, \|F_z\|) / (½ρU²πR²), at n₁ | ≤ 0.002 | ≤ 0.002 | ≤ 0.002 | ≤ 0.002 |
| The drag \|F_x\| / (½ρU²πR²), at n₁ | report only | report only | report only | report only |
| The first measure's change from n₁ to n₂ | report only | report only | report only | report only |
| Order check, samples fixed at r = 4R/3 | | e(12) ≤ 0.7 e(6) | | report only |

**What the C_p band is built from** (potential flow's ∂C_p/∂R at the samples peaks at 1.53/R at R/dx 6 and 2.48/R at 12):
- **The wall.** Halfway, dx/2: 0.128 and 0.104. Bouzidi, 3dx²/(8R): 0.016 and 0.0065.
- **Second order:** 0.061 and 0.015. This is the solids note's term for the same field, one cell out, kept as a bound at two.
- **The boundary layer's displacement**, δ* = 0.0148R: 0.023 and 0.037. It is the flow's own viscous response at Re 2000, not a grid error, so it does not shrink with dx.
- **The Mach term**, 1.64 Ma² = 0.003. That is isothermal Bernoulli's (Ma²/4) C_p² at |C_p| ≤ 1.25, plus the field's own compressibility, taken as Ma² |C_p|.
- **The box's blockage**, (R/8R)³: 0.002. ρ's fp32 rounding in C_p: 0.002.

**The other measures:**
- **The side forces:** no source. The configuration is mirror-symmetric in y and z, so only the summation's rounding of the dynamic part is left, with the rest state off (above), under 10⁻³.
- **The drag** at n₁ is an impulsive, viscous start, which has no closed form. It is printed, not checked.
- **The order check** is halfway's. At r = 4R/3 its wall term halves with dx, and e(12) ≈ 0.5 e(6) by the terms above. 0.7 is an observed order of at least 0.5, as the solids note's. Bouzidi's is report only, since its shrinking terms are the size of its floors (the boundary layer, the Mach term and the box).

## L2b. S2's Couette, in the LBM (added)

**Why it is here.** It turns a curved part with the moving-wall term while its links never change, so a miss in the plate (L2c) can be split between the moving-wall term and the refill. It is the NS tier's S2 (the solids note), and it is exact at any Re.

**Reference.** Circular Couette flow (Landau & Lifshitz 1987, *Fluid Mechanics*, §18). The inner cylinder R₁ turns at Ω and the outer R₂ is still: `u_θ = A r + B/r`, with `A = −Ω R₁²/(R₂² − R₁²)` and `B = Ω R₁² R₂²/(R₂² − R₁²)`. The couple per unit length on the inner cylinder is `G = 4π μ Ω R₁² R₂²/(R₂² − R₁²)`.

**Configuration.**
- **The box:** a quasi-2D slab, one node thick and z periodic, R₂ = 2R₁. R₁ = 16 (a 68² slab) and 32 (132²), the axis midway between nodes. Both cylinders are parts from the SDF.
- **The flow:** ΩR₁ = u_lat 0.02 (Ma 0.035). Re = ΩR₁(R₂ − R₁)/ν = 10: ν_lat 0.032 (τ 0.596) at 16, and 0.064 (τ 0.692) at 32. WALE off.
- **The run:** from rest, impulsive, to T = 2(R₂ − R₁)²/ν, by which the slowest viscous mode has decayed by e^(−2π²): 16 000 and 32 000 steps.
- **The walls:** halfway, then Bouzidi, at both sizes.

| Measure | Halfway, R₁ 16 | Halfway, 32 | Bouzidi, 16 | Bouzidi, 32 |
|---|---|---|---|---|
| max \|u_θ − u_θ,exact\| / (ΩR₁), nodes with R₁ + dx ≤ r ≤ R₂ − dx | ≤ 0.11 | ≤ 0.054 | ≤ 0.0092 | ≤ 0.0029 |
| \|G / G_exact − 1\|, the inner cylinder's couple by momentum exchange | ≤ 0.099 | ≤ 0.049 | ≤ 0.0089 | ≤ 0.0028 |
| Order check on the profile | | e(32) ≤ 0.7 e(16) | | e(32) ≤ 0.5 e(16) |

**What the bands are built from:**
- **The wall, in the profile.** ∂u_θ/∂R₁ peaks at (8/3)Ω and ∂u_θ/∂R₂ at (2/3)Ω.
  - Halfway, dx/2 at both walls: (10/3)(dx/2)/R₁, which is 0.104 and 0.052 (the NS tier's S2 term).
  - Bouzidi, 3dx²/(8r) at each wall (inner r_c = R₁, outer R₂): 0.0044 and 0.0011.
- **The wall, in the couple.** d ln G/dR₁ = (8/3)/R₁ and d ln G/dR₂ = −(1/3)/R₁.
  - Halfway: 0.094 and 0.047.
  - Bouzidi: 0.0041 and 0.0010.
- **Second order:** (dx/(R₂ − R₁))², which is 0.004 and 0.001.
- **The Mach term:** the centrifugal pressure varies ρ, and with it μ = ρν, by at most Ma² ln 2 = 0.0008.
- **The order checks:** 0.7 for first order and 0.5 for second, each half its method's order, as the solids note's. Each is reported, not checked, when the coarse error is under a tenth of its band.

## L2c. The rotating plate's torque, against Jeffery's exact couple

**The plan's "against a fine-grid reference".** A finer grid of our own solver is not a reference under the rule. So this note gives the plate an exact reference: a plate-like oblate spheroid in Stokes flow. The plan's fine grid becomes the order check.

**Reference.** Jeffery (1922), Proc. R. Soc. Lond. A 102, 161–179, eq. (36) with the fluid at rest (its f = ξ = 0), and the integrals of its eqs. (8) and (9) with the lower limit at zero (p. 164). The couple on an ellipsoid with semi-axes (a₁, a₂, a₃), turning at Ω about axis 1 in still fluid, opposes the turn, with magnitude

`L₁ = (16πμ/3) (a₂² + a₃²) / (a₂² α₂ + a₃² α₃) · Ω`, where `α_i = ∫₀^∞ dλ / ((a_i² + λ) Δ)` and `Δ = √((a₁² + λ)(a₂² + λ)(a₃² + λ))`.

- **This case's value:** the spheroid (a, a, a/2) turning about an equatorial axis has `L = 14.2205 μ Ω a³`, by quadrature in double.
- **The checks on the formula:** it gives the sphere's 8π μ a³ Ω to 10⁻⁹, and, as c → 0, the thin disk's (32/3) μ a³ Ω about a diameter. The test evaluates the formula in double. It was read from the source's scan (fetch F1, below), by eye, so the lead does the second reading.
- **Why it fits a moving-wall test.** A body turning about a principal axis feels a constant couple in Stokes flow. So the couple's mean over the run is the reference, while the body's section sweeps the lattice: links are rebuilt every step, and nodes are covered and uncovered.

**Configuration.**
- **The body:** the spheroid (a, a, a/2), centred midway between nodes, turning about the x axis, an equatorial axis. a = 8 (a 64³ box) and 16 (128³). The box is periodic, of side 8a. The rim's radius of curvature is a/4, 2 and 4 cells.
- **The flow:** Re = Ωa²/ν = 0.05, ν_lat 1/6 (τ 1), WALE off. The tip moves at Ωa = 1.04e-3 and 5.2e-4.
- **The run:** one revolution, 2πa/(Ωa): 48 300 and 193 000 steps. The couple is averaged over the second half revolution, which is the body's period. The viscous spin-up, a²/ν, is 384 and 1536 steps.
- **The walls:** halfway, then Bouzidi, with the moving-wall term and Physics' refill rule.

| Measure | Halfway, a 8 | Halfway, 16 | Bouzidi, 8 | Bouzidi, 16 |
|---|---|---|---|---|
| \|mean couple / L − 1\|, about the turning axis | ≤ 0.26 | ≤ 0.13 | ≤ 0.12 | ≤ 0.035 |
| The other two couples over L, and \|F\| a / L, each step's worst | ≤ 0.002 | ≤ 0.002 | ≤ 0.002 | ≤ 0.002 |
| The couple's spread over the window, (max − min) / L | report only | report only | report only | report only |
| Order check | | e(16) ≤ 0.7 e(8) | | e(16) ≤ 0.5 e(8) |
| The walls' cost: ns a cell-step for the links' rebuild in the part's box, and its share of the step | | report only (GPU) | | report only (GPU) |

**What the couple's band is built from:**
- **The wall.** From Jeffery's formula with every surface moved out by δ, d ln L/dδ = 3.50/a.
  - Halfway, dx/2: 0.218 and 0.109.
  - Bouzidi, 3dx²/(8 r_c) at the rim's r_c = a/4: 0.082 and 0.0205. That is a worst case, since the rim is a small part of the surface.
- **Second order:** ½(dx/c)² at the semi-thickness c = a/2: 0.031 and 0.0078. The rotlet field u ∝ r⁻² has u″/u = 6/r², and the term is (dx²/12)(6/c²).
- **The refill:** at most its gross rate, 0.035 Re = 0.0018 (section 3 above).
- **Inertia:** the couple is odd in Ω, so the first inertial correction is O(Re²), taken as Re² = 0.0025 with a unit coefficient.
- **The periodic images' rotlets:** (a/L)³ = 0.002. The Mach term is under 10⁻⁶.

**The symmetry measure.** The body and the lattice are mirror-symmetric in x about the centre at every angle, and the spheroid is centrally symmetric. So the other two couples and the net force are zero up to rounding, with the rest state off. A link or a refill that breaks the symmetry shows there.

## L2d. Exact checks (unit tests, no record)

- **A still part, and the same part moving at zero velocity, bit-equal** (the plan).
- **A moving part's links and q at a pose equal a static part's built at that pose, bit for bit.** This is the "rebuilt from the posed SDF each step" rule, checked directly.
- **CPU = GPU to the bit**, with the loads summed in `reduce.hpp`'s order (the plan's rule).

## L3a. The u_τ solve

**Reference.** Spalding's law (1961, J. Appl. Mech. 28): `y⁺ = u⁺ + e^(−κB) [e^(κu⁺) − 1 − κu⁺ − (κu⁺)²/2 − (κu⁺)³/6]`. Given the velocity u at the matching height y and ν, its root u_τ (with u⁺ = u/u_τ and y⁺ = y u_τ/ν) is found in double by bisection to 10⁻¹⁵. The law is the model, so its constants κ and B are configuration, named by Physics before the first run (open item 7).

**Configuration.** Physics' fixed Newton count from its fixed start, in fp32, with `core/fp32_math`'s exp. It runs over Re_y = u y/ν at 61 points, log-spaced from 1 to 10⁷, which spans y⁺ from under 1 to past C5's.

| Measure | Tolerance | Built from |
|---|---|---|
| max \|u_τ / u_τ,ref − 1\| over the 61 points | ≤ 1e-5 | Converged, fp32 leaves a few ulps (3.6e-7). The exp's argument κu⁺ carries its rounding into y⁺ amplified by κu⁺, but d ln y⁺/d ln u_τ ≈ 1 + κu⁺ divides it back out. 1e-5 is over 20 times that. A count too small for the range shows as a miss, not a band |
| u = 0 gives u_τ = 0; −u gives −u_τ; a NaN in gives a NaN out | exact | The tier's NaN rule; the wall stress opposes the slip |

## L3b. C5's turbulent flat plate, wall-modelled

**Reference.** Spec §11.2's C5 turbulent case: White's correlation, `C_f ≈ 0.455/ln²(0.06 Re_x)`, as the spec gives it, with C_f within 10%.
- **Where the form comes from.** White's *Viscous Fluid Flow* is a book, so no fetch reaches it. The form is the spec's, not read here from its source.
- **Its x is measured from a virtual origin.** A turbulent inflow has no leading edge, so x = x_station − x₀, with x₀ named by Physics before the first run and printed in the record (open item 6). Its placement is a term of the band: d ln C_f/d ln Re_x = −2/ln(0.06 Re_x), about −0.17 here, so an x₀ off by δx moves C_f by 0.17 δx/x.
- **Comparing at Re_θ (Coles–Fernholz) would remove x₀, and waits.** No open source read so far states its constants with their scatter (F2, below). Until one does, it is neither checked nor printed.

Two things differ from the spec, for the lead and Physics:
- **The range is narrowed to what fits the cap.** Re_x 10⁶ to 10⁷ at 16 cells across δ would be about 650 M cells; the cap is about 38 M at 112 B a cell. The proposal, sized for Physics to set:
  - the measured region Re_x ≈ 2·10⁶ to 4·10⁶ (Re_θ ≈ 4000 to 6900);
  - 16 cells across δ at its start;
  - about 950 × 100 × 64 cells, 6 M.
  Re_x 10⁷ waits for L4's nests (open item 5).
- **The claim is narrowed to what a correlation can carry.** A pass says C_f is within 10% of White's correlation. It does not say the wall model is within 10% of the true C_f, because the correlations' own uncertainty at these Re_θ is not small, and none has been read from a source with its scatter. The lead's figures, not read here (L19c):
  - other fits put the Coles–Fernholz C at 3.7 to 3.94, against 4.127;
  - the Melbourne floating-element data (doi:10.1007/s00348-016-2168-y) give κ 0.38 and C 3.7 within 3% for Re_θ > 15 000;
  - at Re_θ ≈ 5000 that is a 3 to 4% spread in C_f.
  So a C5 pass supports the wall model to about 14% of the measured C_f, not 10%, and its record says so.

**Configuration, fixed now except where Physics names it:**
- **The flow:** U∞ at u_lat 0.1 (Ma 0.173); WALE on at C_w 0.5; the span periodic, at least πδ wide; the top face zero-gradient.
- **The wall model:** matched 3 cells off the wall (spec §11.3: a fixed 2 to 3), with Malaspinas & Sagaut's slip velocity (2014, J. Comput. Phys. 275). Matching at the first cell carries the log-layer mismatch (Kawai & Larsson 2012, Phys. Fluids 24), which is why the spec moves it off.
- **The inflow:** turbulent, by Physics' method (for example Lund, Wu & Squires' recycling, 1998, J. Comput. Phys. 140). Nothing is measured within 10 δ_in of it, or within 2δ of the outlet.
- **The averaging:** over the span and over time, for at least 10 flow-throughs after 2 discarded.

| Measure | Tolerance | Built from |
|---|---|---|
| \|C_f / C_f,White(Re_x) − 1\| at each of 5 stations, C_f from the mean modelled wall stress, x from the named x₀ | ≤ 0.10 | Spec §11.2's band, kept. The terms bounded here: the time-and-span average's sampling error, under 1% by the window; the Mach term, Ma²/2 = 1.5% (isothermal: the density's variation across the layer); and x₀'s placement, 0.17 δx/x. The rest is the wall model's and the grid's error, which is what C5 exists to measure. So, as C0b's 10% is, this is the literature's band, not one built term by term. The correlation's own uncertainty is outside it (above) |
| The station's Re_θ, from the mean profile | report only | So the result can be set against a Re_θ correlation once one is read |
| The mean u⁺ against the log law at each station; the matching height in y⁺ and cells per δ | report only | Spec §11.2 sets no turbulent profile band |
| C_f against 2 dθ/dx (von Kármán's momentum integral) between stations | report only | A check that the modelled stress and the layer's growth agree |

## The records

**Where they go.** The shape is `01-verification.md`'s, "Validation records" (`AnalysisValidation*` reads them). Each opening line ends with `LBM`, and names the wall (`halfway` or `Bouzidi`), so no LBM record takes an NS record's identity.
- **`config:`** names the box and N, the lattice and collision, WALE or `WALE off`, u_lat and Ma, ν_lat and τ, the steps, the start, the wall rule, and, for moving walls, the momentum exchange's form and the refill rule.
- **`tolerance:`** lines cite `test-docs/2026-11-03-l2-l3-lbm-tolerances.md, fixed before the first run`. The two parser rules in L1's note hold: a measure's value starts with a number, and a report-only measure carries `tolerance: none (report only)`.

| Record | File | Opening line | Measures, in order |
|---|---|---|---|
| L2a | `s1-sphere.md` | `S1 sphere in a uniform stream, R/dx 6, Bouzidi, LBM` (one per size and wall) | max \|C_p − C_p,pot\| at n₁; the side forces; the drag, report only; the change to n₂, report only; at R/dx 12, the order check |
| L2b | `s2-couette.md` | `S2 circular Couette, R1/dx 16, Bouzidi, LBM` | the profile; the couple; at 32, the order check |
| L2c | `l2-rotating-plate.md` (new) | `L2 rotating plate, oblate spheroid c/a 0.5, a/dx 16, Bouzidi, LBM` | the mean couple; the symmetry measure; the spread, report only; at 16, the order check; the walls' cost, report only |
| L3b | `c5-flat-plate.md` (new) | `C5 turbulent flat plate, wall-modelled, Re_x 2e6-4e6, LBM` | C_f at the 5 stations; then the report-only lines |

L2d and L3a are unit tests and leave no record.

## Where they run

- **Opt-in:** every case, with `SPADE_AIRFLOW_VALIDATION=1`, in release, as L1's are. L2c at a 16 and L3b run on the GPU (the path line says so), since the CPU reference would take hours.
- **The gate:** L2a at R/dx 6 (96³, 42 steps) and L2b at R₁ 16 (68², 16 000 steps), Bouzidi, may join it as regression guards, each only if it times under 30 s in debug, under ctest at `-j 1` (as L1's C0a). L2d and L3a are gate tests.

## Reference data and fetches

Nothing in L2 or L3 needs a data file: potential flow, Couette, Jeffery's couple and White's correlation are closed forms that the tests evaluate in double. Two sources are read before the first run, so each formula's provenance is checked from the paper, not from memory (the lead's fetches, L19):
- **F1, read.** Jeffery, G. B. (1922), "The motion of ellipsoidal particles immersed in a viscous fluid", Proc. R. Soc. Lond. A 102, 161–179, doi:10.1098/rspa.1922.0078: the couple's formula (L2c).
  - **The copy:** the Royal Society's public-domain deposit on the Internet Archive, item `philtrans09717112`. The publisher's own PDF answered a bot challenge, and that was not worked around.
  - **The files:** `09717112.pdf` (SHA-256 0b2ba9464ce926aa9c8083d88d495d3e8a49dbd7270655589bd9b8bed364a3c3) and its page scans, `09717112_jp2.zip` (SHA-256 2fdc0f0357f104d40d0fb0cb37a3f13f1aaa16bee0b3f4c1d9a9be2862bcdc56). Each SHA-1 matches the item's own listing. Kept out of the repo, in the main checkout's `.agents/refs/jeffery1922/`.
  - **The reading:** eqs. (8) and (9) and the zero-limit integrals on p. 164 (scan 0003), and eq. (36) on p. 169 (scan 0008). They were read by eye, since the OCR layer is garbled there, so the lead does the second reading. The formula above is eq. (36)'s L with f = ξ = 0. Its sphere limit, 8π μ a³ Ω, is Stokes's couple.
- **F2, not read: closed access.** Nagib, H. M., Chauhan, K. A. & Monkewitz, P. A. (2007), "Approach to an asymptotic state for zero pressure gradient turbulent boundary layers", Phil. Trans. R. Soc. A 365, 755–770, doi:10.1098/rsta.2006.1948: the Coles–Fernholz constants and their scatter about the data (L3b). OpenAlex lists no open copy, and the publisher's page answered a bot challenge. The lead then approved two open sources from the same group (L19b), and neither could be reached:
  - Monkewitz, Chauhan & Nagib (2007), Phys. Fluids 19, 115101, for κ and C: AIP's open copy answered the same bot challenge, which was not worked around. EPFL's record (179698) holds a licence file and no text.
  - Chauhan, Monkewitz & Nagib (2009), Fluid Dyn. Res. 41, 021404, for the scatter: EPFL's record (159338) holds a licence file and no text.
  - **The lead then approved two arXiv papers (L19c),** each read once from arxiv.org and kept in the main checkout's `.agents/refs/`. Neither states the constants with their scatter:
    - Yakhot, V., "Skin friction in simple wall-bounded shear flows in large Reynolds number limit", arXiv:0909.4387v3 (SHA-256 e4f41c3de752e2049edf093a26437dec498071b2b26bfeed3cef9ab55b807722). It cites Nagib et al. (2007) and Fernholz & Finley (1996, Prog. Aerosp. Sci. 32) for the Coles–Fernholz relation. It writes the relation only as C_f ∝ 1/ln² Re_δ, with no κ, no C and no scatter.
    - Xia, Z., Zhang, P. & Yang, X. I. A., "On skin friction in wall-bounded turbulence", arXiv:2007.14619v1 (SHA-256 d4a5e36b839525a31a1c04025dbd1fc6fa007cb76017b0d80c0a5d3a4c831210). It uses the log law, not Coles–Fernholz. Its one figure on the constants' effect is Crook's (2002, CTR Annu. Res. Briefs): a change of about 0.5 in the log law's slope 1/κ gives a 12% difference in u_τ.
  - **So L3b takes the spec's White's correlation, and Coles–Fernholz waits** (the lead's rule, L19c).

## Open, for Physics before the first run

1. **The loads' rest state** (section 2 above). The bands assume each link's rest state comes off before the sum, by shifted storage or by subtraction. Without that, fp32 rounding is about the size of L2a's side-force band and a quarter of the plate's couple, and the same holds for L5's blade loads.
2. **S2's Couette is added** as L2b, to split the plate's moving-wall term from its refill. Confirm, or drop it.
3. **The plate's reference** is Jeffery's exact couple on a spheroid (a, a, a/2), not a fine grid. A thinner plate, c/a 0.25, would put its rim radius at one cell at a 16. Its Bouzidi term would then be 0.081, and its band about 0.12.
4. **The moving wall's momentum exchange and refill rule:** name both before L2c's first run. Each record's `config:` line prints them.
5. **C5's domain and inflow:** the method, the 10 δ_in recovery length, and the sizes above, about 6 M cells for Re_x 2·10⁶ to 4·10⁶. The spec's Re_x 10⁷ waits for L4.
6. **C5's virtual origin x₀:** Physics names how x₀ is placed before the first run, and the record prints it. Comparing at Re_θ (Coles–Fernholz) would remove x₀, and waits for a source that states its constants with their scatter.
7. **Spalding's constants and the Newton count:** name κ and B (Spalding's own are 0.4 and 5.5), the start and the count, before L3a.
8. **No case in L2 checks a viscous force against published data:** L2a reads the pressure early, and the plate is Stokes flow. C2 (spec §11.2: Dennis & Chang 1970, the steady cylinder at Re 20 and 40, C_d and L/D within 5%) would. Consider it for L2 or before L5. Its data would then need a fetch.
