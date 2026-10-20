# The solids tests (P2, P3): tolerances fixed before the first run

**Owner:** Test/Docs. **Status:** draft for the lead's review, 2026-10-13. None of these tests has run yet.
**For:** the tests in §8 of Physics' solids design (`../physics/plans/2026-10-13-airflow-solids-design.md`, approved 2026-10-13). The tests read their tolerances from here, and each record's `tolerance:` line cites this note.
**The rule** (`01-verification.md`, "Validation against published data"): a tolerance against a reference is fixed before the first run, from the reference and the method, never from our solver's own error. It is not loosened after a run without a lead decision recorded with its reason. This note replaces test 1's "band measured at 12 cells per diameter".

## How the tolerances are built

Every reference here is exact: an analytic solution of the equations the solver discretises, or the porous model's own law. So a tolerance is the method's own error at the stated resolution, estimated term by term from the exact solution:

- **The wall is first order.** The aperture ramp `θ = clamp(0.5 + φ/dx, 0, 1)` (spec §2; Batty, Bertails & Bridson 2007, "A fast variational framework for accurate solid-fluid coupling", ACM Trans. Graph. 26(3)) spans one cell, so it places the wall to within half a cell. A sheet's binary faces (the design's §10, decision 2) place the sheet to within half a cell too. A measure's wall term is its sensitivity to a wall moved by `dx/2`.
- **P3 reads a cut cell's own pressure,** which may lie up to half a cell off the wall. A force's sampling term is its sensitivity to that offset.
- **The interior is second order** (MAC central differences, MacCormack advection, trilinear sampling). Its term comes from the exact solution's second derivatives at the stated resolution.
- **A steady measure carries no iteration error.** Each step's V-cycles and viscous sweeps start from the last step's state, so a steady state of the step is the converged discrete solution, whatever `n_v` and `visc_sweeps` are. A measure taken in a transient names a cycle count that converges from a cold start; a solve that does not converge shows as a miss.
- **A tolerance is the sum of its terms,** each a worst case, rounded up to two figures. A correct solver passes with room. A sign error, a factor of two, a leak or a wall in the wrong cell does not.
- **Where it is cheap, an order check** runs the case at `dx` and `dx/2`. The fine grid's error must be at most 0.7 of the coarse grid's: an observed order of at least 0.5, half the method's first order, since two grids pin the order no closer. It is reported, not checked, when the coarse error is already under a tenth of its tolerance.

## S1. A sphere in a uniform stream (design test 1)

**Reference.** Potential flow past a sphere of radius `R` in a stream `U` (Batchelor 1967, *An Introduction to Fluid Dynamics*, ch. 6), evaluated in double at each sample point. With `a = (R/r)³`: `u_r = U cos ϑ (1 − a)`, `u_ϑ = −U sin ϑ (1 + a/2)`, `C_p = 1 − |u|²/U²`. On the surface that is `1 − (9/4) sin²ϑ`. The steady force is zero (d'Alembert). Started impulsively, the same flow gives the sphere an impulse of half its displaced mass times `U` (added-mass coefficient `C_M = ½`).

**Configuration.** Inviscid, with the free-slip option. A cubic box of side `8R` with the sphere's centre at its centre, which is a cell corner, so the apertures are mirror-symmetric on all three axes. Inflow `U` on −x; the other faces open. `U h/dx = 0.1`, `n_v = 16`. With solids the projection runs CG preconditioned by a V-cycle, and `n_v` counts its iterations (Physics, 2026-10-13); the solve must reach 10⁻⁴ of its starting residual, and the record prints it. `R/dx = 6` (the design's 12 cells per diameter), and `R/dx = 12` for the order check.

**When it is read: before a wake can form.** An inviscid run on a first-order wall separates numerically on the lee side once the flow has moved a fraction of a radius, and from then on potential flow is the wrong reference.
- The added-mass impulse is read at step 1. Advecting a uniform field is exact (spec §1.5), so step 1's projection alone turns the uniform stream into the discrete potential flow, and its pressure is the impulse over `h`.
- `C_p` and the force are read at step 3 (`t U/R` = 0.05 at `R/dx` 6). From step 2 on, the pressure balances the advection of the potential flow, which is Bernoulli's pressure.

**The samples.** `C_p` is read one cell out along the outward normal (RND-8's rule, trilinear in the cell-centre pressure), every 15° in ϑ on four meridians, and compared with potential flow at the same points, so the sample's offset is not an error.

| Measure | `R/dx` 6 | `R/dx` 12 | Built from |
|---|---|---|---|
| The largest `\|C_p − C_p,pot\|` over the samples (step 3) | ≤ 0.32 | ≤ 0.20 | Wall: `∂C_p/∂R` at the samples peaks at 2.5/`R` (6) and 3.3/`R` (12) at ϑ = 90°, so a wall `dx/2` off gives 0.21 and 0.14. Second order: 0.06 and 0.015. The splitting's first-order time error at `U h/dx` = 0.1: 0.03 and 0.02. The box's blockage, `(R/4R)³` of the disturbance: 0.02 |
| The drag `\|F_x\| / (½ρU²πR²)` from P3 (step 3) | ≤ 0.06 | ≤ 0.04 | The apertures are fore-aft symmetric, so the wall, sampling and blockage terms cancel in the drag. What is left is the advection's upwind asymmetry, at most the splitting term, doubled because `∮\|n_x\| dA = 2πR²` |
| The side forces `\|F_y\|`, `\|F_z\|` over `½ρU²πR²` | ≤ 0.01 | ≤ 0.01 | Axisymmetric, so no first- or second-order source: only rounding and the red-black order, both under 10⁻³ |
| The added mass `C_M = F_x h / (ρ (4/3)πR³ U)` from P3 (step 1) | 0.5 ± 0.24 | 0.5 ± 0.12 | Relative: the volume the wall encloses, `3 (dx/2)/R`; P3's pressure read up to `dx/2` off a field falling as `1/r²`, `dx/R`; second order and the box, 0.05 (6) and 0.025 (12). Times 0.5 |
| Order check, `C_p` at samples fixed at `r = 7R/6` | | e(12) ≤ 0.7 e(6) | |

d'Alembert's zero cannot fail on its own. On a closed body the cut cells' area vectors sum to zero exactly, so an integral that returned zero would pass. The added-mass impulse is a non-zero force for the same integral. The design's flux and divergence checks stay as written: closed faces carry the wall velocity exactly, and the divergence keeps plan A's band.

## S2. A rotating cylinder: circular Couette flow (design test 2)

The design's case, a cylinder spinning in still air, has no closed form before its steady state, and that steady state (`u_θ = Ω R²/r`) needs an unbounded box and unbounded time. Inside a fixed coaxial tube the same cylinder has an exact steady solution, so the measure is taken there. The design's two checks stay, as exact ones: closed faces carry the rigid velocity bit for bit, and the air's circulation has the rotation's sign. **Physics to confirm the change.**

**Reference.** Circular Couette flow (Landau & Lifshitz 1987, *Fluid Mechanics*, 2nd ed., §18). The inner cylinder `R₁` turns at `Ω` and the outer `R₂` is still: `u_θ = A r + B/r`, with `A = −Ω R₁²/(R₂² − R₁²)` and `B = Ω R₁² R₂²/(R₂² − R₁²)`. It solves the full Navier–Stokes equations at any `Re`.

**Configuration.** Quasi-2D (one cell in z, periodic). `R₂ = 2R₁`. The tube is a static wall part and the cylinder a turning one, both no-slip. `Re = Ω R₁ (R₂ − R₁)/ν = 10`, far below Taylor's vortices (Taylor 1923, Phil. Trans. R. Soc. A 223), which a one-cell slab cannot carry anyway. `Ω R₁ h/dx = 0.5`. Run to `T = 2 (R₂ − R₁)²/ν`, by which the slowest viscous mode has decayed by `e^(−2π²)`. `R₁ = 32 dx` (a 136² slab), and `R₁ = 16 dx` for the order check.

**Measure.** The largest `|u_θ − u_θ,exact| / (Ω R₁)` over the cell centres with `R₁ + dx ≤ r ≤ R₂ − dx`, from the centred velocity.

| | `R₁/dx` 16 | `R₁/dx` 32 | Built from |
|---|---|---|---|
| Tolerance | ≤ 0.11 | ≤ 0.054 | Wall: `∂u_θ/∂R₁` peaks at `(8/3) Ω` at the inner wall and `∂u_θ/∂R₂` at `(2/3) Ω` at the outer, so both walls `dx/2` off give `(10/3)(dx/2)/R₁` = 0.104 and 0.052. Second order, `(dx/(R₂ − R₁))²`: 0.004 and 0.001 |
| Order check | | e(32) ≤ 0.7 e(16) | |

**The assumption this rests on.** The design says how the projection treats a wall's normal flux, but not how the viscous stencil treats the tangential velocity. Spec §11.3 item 3 puts the tangential condition on the same apertures, which places the no-slip wall within half a cell, as above. If the viscous stencil instead takes the rigid velocity only at closed faces (θ = 0), the no-slip wall can sit up to a cell inside the solid. The case can then miss by up to a factor of two, while its order check still passes.

**Measured: S2 does not tell the two treatments apart** (Physics, P2, 2026-10-20). At `R₁/dx` 16, S2 passes with the viscous wall links at 0.0052, and also with them switched off (the rigid velocity at closed faces only) at 0.0602. Both are inside 0.11, though the links are 12 times better. So S2 stays as it is, a check of the solution, and the treatment gets a check of its own:
- **The discriminating check is P2's planar Couette**, with both plates a quarter cell off the face planes. With the links, the exact linear profile is the discrete fixed point, held to rounding. With closed faces alone, each wall sits 0.75 dx too far and the profile drifts.
- **Its band** is measured and pinned (`TD-2`), and the record carries both treatments' numbers, so the band visibly fails the links-off run.
- **Where S2 runs:** about 15 s in release, about 100 s under debug ctest (whose limit is 60 s). It runs in the release gate and every leg. A skip in debug, by name, with the planar check covering debug, waits on the lead's ruling under `TD-5`.

## S3. A sheet across a channel (design test 3)

**Reference.** A channel of length `L`, periodic along x and holding a uniform `u = U`, is blocked across its whole section by a sheet. One projection must stop the flow, `u ≡ 0`. The pressure impulse `p h` then rises along x at `ρU` per metre and drops by `ρ U L` across the sheet, which is 1D and exact. The force on the sheet is `ρ U L A / h`, with `A` the section.

**Configuration.** Inviscid, `L = 32 dx`, a section of 8 × 8 cells periodic in y and z, and a plate 0.25 `dx` thick across it (a sheet by the design's §4 rule). One step, `n_v = 32` CG iterations from a cold start. Physics' check that both parities converge to 10⁻³ within 8 iterations is an implementation band under `TD-2` (measured at 4 in its port), not one of these tolerances. It runs twice, with the sheet on an even fine face plane and on an odd one. The multigrid's coarse faces lie only on even fine planes, so a sheet on an odd plane is invisible to its coarse levels (finding 3).

| Measure | Tolerance | Built from |
|---|---|---|
| Every closed face carries the wall velocity, 0 | exact | The design: a face with θ = 0 takes `u_s` |
| `max \|u\| / U` after the step | ≤ 10⁻³ | Exactly 0 in 1D. What remains is the solve's convergence |
| The pressure step between the cell centres either side, `Δp h / (ρ U (L − dx))` | 1 ± 10⁻³ | The stencil is exact for a linear field, and the exact solution's step between centres `dx` apart across the sheet is `ρ U (L − dx)/h`. Rounding is about 10⁻⁵ |
| P3's force on the sheet, `F h / (ρ U L A)` | 1 ± 0.033 | P3 takes each side's pressure at the next cell centre, half a cell off the sheet: a first-order term `dx/L` = 0.031 (the scheme gives `1 − dx/L`), plus 10⁻³ |
| The run line names it a sheet, with its thickness | exact | |

## S4. A porous block (design test 4)

**Reference.** The model's own law (the design's §5, after Darcy 1856 and Forchheimer 1901). A uniform stream `U` through a slab of length `L` filling the channel's section loses `Δp = ρ L (ν U / K + ½ C₂ U²)`.

**Configuration.** A channel along x, 64 × 4 × 4 cells, with inflow `U` on −x, an outlet on +x, and periodic y and z, so the flow is 1D and `u = U` everywhere after any projection. A block `L = 8 dx` long fills the section, at a non-integer offset. `ν > 0`, which the Darcy term needs. Three cases, at Forchheimer numbers `½ C₂ U K / ν` = 0.1, 1 and 10. `h` is chosen so `h c ≤ 10⁻³`, with `c = ν/K + ½ C₂ U` (see below). It is read after 10 steps; the flow is steady from step 1.

| Measure | Tolerance | Built from |
|---|---|---|
| `Δp` between the cell centres two cells outside each end, over the law's `Δp` | 1 ± 0.005 | The slab's sink is exact: for a face-normal end at any offset, the ramp `s` sums to `L/dx` over the faces. The uniform flow has no other pressure gradient. The splitting factor `1/(1 + s h c)` is at most 10⁻³ off by the configuration. Rounding and the warm solve are under 10⁻⁴ |
| The same at `h c = 1` | 1 ± 0.005 | Finding 1 is taken up (Physics, 2026-10-13): with `β = 1/(1 + s h c)` in the projection's face weight, the steady drop is exact at any `h`, so the same terms apply |

**Why `h c` is held small.** The sink is applied before the projection, and the projection restores the velocity the sink took. At a steady state `u = u/(1 + s h c) − (h/ρ) ∂p/∂x`, so the pressure gradient is `ρ s c u / (1 + s h c)`, not `ρ s c u`. The drop is low by the factor `1/(1 + s h c)`: a first-order splitting error in `h` that does not go away at steady state.

## S5. Publish beside a sheet (design test 8)

**Reference.** Exact: a velocity field that is linear on each side of a sheet. The sheet lies in the plane `y = y_s` and runs along x and z, both periodic. Above it, `u = U_a + S (y − y_s)` and `w = W_a`; below, `u = U_b + S (y − y_s)` and `w = W_b`; `v = 0` throughout. The jumps `U_a − U_b = U` and `W_a − W_b = U` are what a free-slip sheet carries, and `S dx = 0.01 U`. Everywhere the true `|ω| = |S|` and `Q = 0`.

**Why the jumps.** The design's field, one uniform shear across the sheet, is continuous there, so a central difference across the sheet also gives `S`. The test could not tell the one-sided rule from the rule it replaces. With the jumps, a central difference would publish `|ω| ≈ U/(√2 dx)`, about 70 `S`.

**Configuration.** The field is written into the grid, and publish is called on it with the sheet's closed faces. No step is taken.

| Measure | Tolerance | Built from |
|---|---|---|
| `\|ω\|` in the cells next to the sheet, against `S` | relative 10⁻³ | A one-sided difference is exact for a linear field. fp16's rounding is at most 2⁻¹¹ (4.9 × 10⁻⁴); fp32's cancellation in the difference, under 10⁻⁴ |
| `q̂` next to the sheet | `\|q̂\| ≤ 10⁻³ S` | `Q` is 0 exactly. fp32 rounds it to about `ε S²`, which `q̂ = sign(Q) √\|Q\|` turns into about `√ε S` ≈ 3.5 × 10⁻⁴ `S` |

In a tangential jump, a central difference fakes `|ω|` but leaves `Q` and λ₂ at 0, since the gradient it adds is a pure shear. So `|ω|` is the channel that tells the rules apart. λ₂ is drawn by Rendering's views, whose gradient needs the same closed-face rule; that is Rendering's half.

## The other tests (design tests 5, 6, 7)

- **Refusals (5):** exact. The refusal names the part and the `dx` it needs, or the floor height and the part it cuts.
- **No solid, no change (6):** both goldens bit-identical (`TD-1`).
- **The posed drone from A4 and from B4 (7):** an implementation comparison, two evaluations of one box, not a physics one. So its band is measured and pinned under `TD-2`. Both distances are exact for a box, so a difference in θ beyond the distance's fp32 rounding (a few ulps of the coordinates, over `dx`) is a bug, not a band.

## Records

S1 to S4 leave records in the shape of `01-verification.md` ("Validation records"), in `tests/data/airflow/records/`: `s1-sphere.md`, `s2-couette.md`, `s3-sheet.md` and `s4-porous.md`. The opening line names the case and its configuration, for example `=== validation record: S1 sphere in a uniform stream, R/dx 6 ===`. Each `tolerance:` line cites this note, for example `<= 0.32 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)`. The order checks print their own tolerance, or `report only` below their floor, as above. S5 is a unit test and leaves no record.

S1 at `R/dx` 12 (96³, three steps) is opt-in under `SPADE_AIRFLOW_VALIDATION`, like V1's opt-in cases. The rest look small enough for the gate, which Physics' timings settle.

## What the analysis found (for Physics)

2026-10-13: Physics took up all six, before any run: β in the face weight (1); CG preconditioned by a V-cycle whenever solids exist (3; spec §1.5 had rejected it, so the lead rules on the spec); a mean per closed region, a closed region with a net flux refused, and a new test 9 (4); S5's jumps and S1's added mass (5); and no-slip by a ghost-fluid Dirichlet on φ = 0 (6; Gibou et al. 2002).

1. **The porous sink's steady drop depends on `h`** (S4). Split before the projection, the point-implicit sink leaves a steady drop of `1/(1 + s h c)` of the law's. Take a screen with loss coefficient `k` = 1, modelled as a block 2 cells thick at `dx` = 6 mm: in a 5 m/s stream it has `½ C₂ U` ≈ 200 per second, so at `h` = 1 ms its drop is 17% low. A foam with `K` near 10⁻⁷ m² adds `ν/K` ≈ 150 per second. The fix is to divide the projection's face weight by `(1 + s h c)`, `u = (u* − (h/ρ)∇p) / (1 + s h c)`, the porous counterpart of θ. That makes the steady drop exact at any `h`. Otherwise `h c` needs a refusal or a label. S4 holds `h c ≤ 10⁻³` either way.
2. **A sphere separates numerically** (S1). An inviscid run on a first-order wall grows a lee-side wake, and potential flow describes it only before that, hence S1's early readings. A late reading would compare against the wrong reference.
3. **A sheet on an odd face plane is invisible to the multigrid's coarse levels** (S3). A coarse face's θ is the mean of the fine faces it covers, and those lie on even fine planes only. A sheet on an odd plane leaves the coarse grid open, so the coarse correction pushes flow through it and the smoother has to undo that, which can stall the cycle. S3 runs both parities. The usual cure is coarsening that follows the operator, which is Physics' call.
4. **A sheet or solid can split the fluid into separate regions.** The design removes the mean over all the fluid cells together (§3), but each separated region needs its own: a closed region whose walls take a net flux is ill-posed on its own, and `check_domain` sees only the domain's faces. S3's periodic channel stays one region. A refusal by name, like `check_domain`'s, would cover the rest.
5. **Two designed checks cannot fail on their own.** Test 8's continuous shear passes under either difference rule (S5's jumps fix that), and test 1's zero force passes for an integral that returns zero (S1's added mass fixes that).
6. **The viscous stencil's tangential condition at an immersed wall is not in the design** (S2's assumption).
