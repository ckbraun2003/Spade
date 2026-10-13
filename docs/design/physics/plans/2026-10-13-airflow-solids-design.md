# Solids in the airflow grid: P2, P2b and P3 — design

**Owner:** Physics. **Status:** **APPROVED by the lead, 2026-10-13**, with §10's two decisions. **Amended 2026-10-13** for Test/Docs' tolerances and the lead's five findings (§11, in the lead's review; the solver change in §3 reverses spec §1.5's rejection of CG and needs the lead's approval). It implements the demonstration plan's P2 (immersed solids), P2b (the drone box's boundaries) and P3 (wall pressure) on plan A's CPU reference. The GPU twin follows in plan B's kernels.
**Rests on:**
- the airflow spec §2 (Boundaries) and §1.5 step 6 (the projection with apertures);
- the demonstration plan (`PHY-10`, the drone-box premise of `cd298d6`);
- Core's parts: `CORE-7`; `vehicles/part_pose.hpp`'s `sample_parts()`; meshes as parts (B4); and A4, the airframe compile's wall parts;
- the lead's rulings of 2026-10-12: build on `sample_parts()` from the start; thin walls become labelled sheets, never silently porous and never silently dropped.

## 1. What a user gets

The drone in the box is a solid wall. Air goes around it, not through it. Its surfaces show the solved pressure. The force and moment the air puts on it are integrated, for display (one-way). Every part's geometry, whether compiled from the airframe or imported as a CAD mesh, reaches the grid the same way. A part the grid cannot resolve as a wall is either a labelled sheet or refused by name, with the `dx` it would need. It never quietly becomes something else.

## 2. The geometry, read through `sample_parts()`

- **One query per point.** `sample_parts(model, posed, x)` gives the signed distance (negative inside), its gradient, the owning part, its surface kind (wall or porous) and the owner's wall velocity at `x`. Every geometric quantity below comes from it. `posed_program()` is never used: it refuses mesh parts, and import is the normal case (Cameron's Q2).
- **Posing.** `posed_parts(model, body, rigid_only)` for the disc and line tiers, which model the rotor themselves; `all` for blade-resolved (D3). In the drone box the body never translates, so the pose changes only with attitude, and with the blade azimuth for rotor parts.
- **The support box.** Only faces within the posed parts' bounding box, grown by 2 `dx`, are sampled. Every face outside is open (θ = 1).
- **When it is rebuilt.** Rigid parts: whenever the body's orientation changes (compared bit for bit), otherwise kept. Rotor parts (D3): every fluid step.

## 3. Walls: face apertures (spec §2, Batty, Bertails and Bridson 2007)

- **The open fraction of a face** at its centre: `θ = clamp(0.5 + φ / dx, 0, 1)`, with φ = `sample_parts(...).distance` when the owner is a wall part.
- **The projection** solves `∇·(w∇p) = (ρ/h) ∇·(w u* + (1 − θ) u_s)`, with each face's weight `w = θ β`: θ its open fraction, β its porous factor (§5; 1 off porous parts):
  - each face's weight w multiplies its neighbour term in the stencil and in the diagonal;
  - `u_s` is the owner's wall velocity component normal to the face;
  - after the solve, a face with θ > 0 takes `β (u* − (h/ρ)∇p)`, and a face with θ = 0 takes `u_s`.
- **Bit identity.** With no solid the stencil is plan A's, bit for bit: `1.0f * p` is `p` and the six weights sum to exactly 6. So both goldens hold, and the solids path is taken only where a w < 1 exists.
- **The solver with solids** (the lead's finding 2, amended 2026-10-13). Plain V-cycles stall on a sheet the coarse levels cannot see:
  - A coarse face covers only the fine faces on its own plane, every second fine plane. So a sheet on any other plane is invisible to that level and every coarser one, and at some level that is every sheet.
  - Measured on a port of plan A's cycle (S3's channel, 32 × 8 × 8, all periodic; Physics' ledger, `spade-wt/physics/.superpowers/sdd/airflow-solids/mg_sym_port.py`): with the sheet on fine plane 16, which every level sees, 21 cycles reduce the residual to 6·10⁻⁸ of its start. On planes 17 and 18 they leave 0.3 and 0.06.
  - Moving an odd plane's closure onto a coarse face, by a min or a series rule, makes the cycle diverge, because the transfers then disagree with the coarse operator.
  - **The fix:** with any w < 1 in the grid, the projection runs CG preconditioned by one symmetric V(2,2) cycle (MGPCG; McAdams, Sifakis and Teran 2010, "A parallel multigrid Poisson solver for fluids simulation on large grids", SCA):
    - The cycle is symmetric. The restriction is the transpose of plan A's trilinear prolongation, halved (full weighting). Pre-smoothing is red then black, post-smoothing black then red, the coarsest level takes 16 sweeps each way, and every level's p starts at 0.
    - With plan A's restriction (the children's sum) the cycle is not symmetric (10⁻¹ measured with a sheet), and CG on it fails even on plane 16.
    - Coarse weights stay the mean of the fine faces each coarse face covers (spec §1.5).
    - `n_v` CG iterations, a fixed count from the last step's p; nothing stops on a residual. Each iteration takes one cycle, one operator application and two dot products. The dot products use `reduce.hpp`'s fixed-tile sum, so the CPU and the GPU twin sum in one order.
    - Each closed region's mean is removed from r and z every iteration (next bullet).
  - Measured on the same port:
    - The sheet on planes 16, 17 and 18: 1–2·10⁻³ of the starting residual in 4 iterations, and under 10⁻¹³ in 12.
    - A 6 × 6 plate on an odd plane: 2·10⁻⁴ in 4.
    - Two perpendicular plates: 5·10⁻⁶ in 8.
    - A prolongation that does not interpolate across closed coarse faces was measured too. Under CG it changes nothing, so it is left out.
  - Without solids, plan A's plain V-cycles stay, bit for bit.
  - **The spec change** (**APPROVED by the lead, 2026-10-13**; spec §1.5 amended): spec §1.5 rejected multigrid-preconditioned CG for the first cut because of the dot products' order between the CPU and the GPU. `reduce.hpp`'s fixed tiles (spec §4.5 as amended 2026-10-12) now fix that order, as P3's force already relies on. CG's scalars still feed back into every cell, so the GPU twin's band on p (TD-14) is measured on S1 and S3, not assumed.
  - **The lead's conditions:**
    - **The iteration count** is fixed, as plan A's cycle count is. It is chosen from S1, S3 and a dense-parts case (the reference quad's parts at the D1 nest), and the choice and its measurements go in this section.
    - **The residual.** Each projection's final relative residual, `max |r| / max |b|`, goes on the run line and into history. A max is order-free, so both backends give the same value. Above the stated threshold, first 10⁻³ and revised with the count, it is announced (`L6`), never silent.
    - **Symmetry.** A test checks the preconditioner's symmetry, `⟨Mx, y⟩ = ⟨x, My⟩` to rounding on random vectors (§8, test 10).
  - **The cost:** one iteration costs about 1.3 V-cycles.
- **Fluid regions** (the lead's finding 3). A sheet or a solid can cut the fluid into separate regions, and each closed one has its own null space.
  - **Labelling.** At each rebuild of the apertures, the fluid cells are labelled into connected regions. A fluid cell is one with an open face (w > 0): to another cell, through a periodic seam, or onto a pressure outlet. The labelling is a flood fill across open faces. It starts from each unlabelled cell in array order and visits faces in the stencil's order, so the labels are deterministic.
  - **Pinned and closed.** A region with a pressure-outlet face is pinned and has no null space. A closed region has its mean removed from b once, and from p after each cycle (from r and z under CG), each by its own masked fixed-tile sum.
  - **The refusal.** A closed region must take no net prescribed flux. The outward normal velocity of the domain faces it touches, its walls and inflows, summed over their samples, must vanish within 10⁻⁶ of their magnitudes, as `check_domain` does for the whole domain (plan A's I1). Otherwise the build is refused by name: the region (one of its cells) and its faces. So a closed region with an inflow and no outlet is refused, even when the whole domain balances.
  - **Immersed walls need no check.** A rigid motion's flux through any closed surface of cell faces is exactly 0, since the face-centre rule is exact for a linear field. Only rounding is left, and the mean removal takes it.
  - **The run line** reports the count, for example "fluid regions: 3 (1 with an outlet, 2 closed)".
  - Cells with no open face are solid, in no region, and keep p = 0.
- **The viscous stencil at an immersed wall** (the lead's finding 5). The implicit viscous solve imposes no-slip with the owner's rigid velocity (decision 1) on φ = 0, where the apertures put θ = ½. It uses the symmetric ghost-fluid Dirichlet treatment (Gibou, Fedkiw, Cheng and Kang 2002, "A second-order-accurate symmetric discretization of the Poisson equation on irregular domains", J. Comput. Phys. 176):
  - **Inside the wall.** A velocity sample with φ < 0 at its own position takes the owner's rigid velocity component there and is not solved. A θ = 0 face so carries the projection's `u_s`.
  - **A crossing link.** A solved sample (φ ≥ 0) may have a link to a neighbour with φ < 0. That link crosses the wall at the fraction `f = φ_self / (φ_self − φ_nb)` of its length, clamped to at least 0.01. Its term `α (u_nb − u)` becomes `(α/f)(u_w − u)`, with `u_w` the owner's rigid velocity component at the crossing, and the sample's diagonal gains `α/f` in place of `α`.
  - **Sheets.** A link the sheet crosses (§4's test, applied to the link) takes the same term, with f from the two samples' unsigned distances to the mid-surface. Both sides are solved.
  - **What it gives.** The tangential and the normal condition sit on the same surface, to second order in the wall's position. Samples with no crossing link keep plan A's stencil, so the no-solid path is bit-identical.
  - **φ at the samples.** φ at each face centre is what θ needs anyway, so it is kept per face in the support box.
  - **Free slip.** The free-slip lean mode is inviscid and has no viscous solve.
  - S2 (circular Couette) tests it.
- **Inside a solid.** Backtraces that land in a solid read the owner's rigid velocity there. A wall face is a no-slip, moving wall, and the rigid field is exact inside it. The spec's free-slip lean mode extrapolates instead, two layers by fixed sweeps; it stays an option for coarse grids, named in the configuration.
  - **How** (approved by the lead, 2026-10-13). Every closed sample (θ = 0) is written with the owner's rigid velocity at each boundary application, and the backtrace interpolates as usual. Deep in a solid that is exact, since trilinear interpolation reproduces the linear rigid field. Within a cell of the surface it blends with the fluid side, as the wall's own first-order placement does. No per-backtrace geometry query is needed.
  - **The free-slip option.** It instead fills two layers of closed samples by fixed Jacobi passes, each from its open (then first-layer) neighbours. Deeper samples keep the rigid velocity. It is inviscid, so it is refused with ν > 0.
- **The layering** (2026-10-13). `spade_vehicles` links `spade_physics`, so the airflow core cannot include `vehicles/`. It takes its solids as data:
  - a sampler callback: a grid-frame point to distance, gradient, part and rigid velocity, for the nearest part (`sample_parts()`'s rule) and for one part alone;
  - per part, its name, surface kind, declared `min_feature`, part-frame bounds extent and posed bounding box.
  The callers adapt `vehicles::sample_parts()` and `part_bounds()` to it: the tests, the D1 demo and plan C's module. The airflow core holds no bounds code (TD-9).
- **The minimum resolution (D5).** A wall part's `min_feature` must be at least 2 `dx`. Section 4 says what happens below that.

## 4. Thin walls: labelled sheets (the lead, 2026-10-12)

The drone's frame plates and boards (1.5–4 mm) are thinner than 2 cells at D1's 6 mm nest.

- **The rule.** A wall part whose `min_feature` is below 2 `dx`, while its other two extents are at least 2 `dx`, becomes a **sheet**: zero thickness, at its mid-surface. In practice that is a plate, a board or a fin.
- **Which faces it closes.** A face is closed (θ = 0) when the sheet crosses the segment between the face's two cell centres: φ at the face centre is below `dx/2 · |n · e_face|`, with n the distance gradient. Otherwise the face is open. So a sheet is impermeable to first order, with a staircase surface. It cannot leak at half-open faces the way a sub-cell solid on the ramp would.
- **How it is labelled.** The run line and the records name each sheet part and its true thickness, for example "frame_top: sheet (2.0 mm < 2 dx = 12 mm)". Its pressure is integrated on both sides (§6).
- **What is refused by name.** A part below 2 `dx` in more than one extent, such as a wire or a small fastener, is refused with the `dx` it needs. It is never porous and never dropped. Interface's panel shows the refusal and the grid that would resolve the part.
- These thresholds are first values. Validation revises them: a flat plate normal to the flow, compared with the published drag against `Re`, is V2's case.
- **The classifier** (amended 2026-10-13, the lead's ruling). A part is thin when `min(declared min_feature, its smallest part-frame bounds extent) < 2 dx`, with the bounds from Core's `part_bounds()`. The extent is the cheap catch for an axis-aligned plate declared thick. One function, `classify_part()`, applies the rule. Interface's `resolve()` calls it instead of keeping its own two-cell rule (TD-9).
- **The measured safeguard** (amended 2026-10-13; this replaces "interior cell centres against volume / dx³"). That comparison detects nothing: the count of cell centres inside any shape is an unbiased estimate of its volume / dx³, thin or not.
  What actually leaks is a wall part that some grid line crosses without meeting a closed face. So at each grid build, per wall part, along each axis:
  - **A run** is a stretch of cell centres along a grid line. It starts and ends at centres outside the part (φ > 0), with k ≥ 0 centres inside it (φ ≤ 0) between them.
  - **The k = 0 screen.** A run with no centre inside can cross the part only where `φ₁ + φ₂ ≤ dx`, since φ is 1-Lipschitz. That holds exactly for meshes and primitives (the heightfield's φ is a conservative bound, also ≤ 1), and union, intersect, subtract and Core's smooth union keep it (`world/sdf.cpp`: the smooth union's gradient is a convex blend of its operands'). Candidates are sphere-traced (`t += φ`, a fixed cap of 32 steps, a hit at `φ ≤ 10⁻⁴ dx`) to confirm the crossing.
  - **Entry and exit.** For every run, a sphere trace from each end finds the points where the line enters and leaves the part, and its outward normals there.
  - **A thin crossing** is one whose normals oppose, `n_in · n_out ≤ −0.5`: a plate, a fin, a blob, or a wedge sharper than 60°. A resolved part's convex 90° edge gives 0 and is not a thin crossing. Without this test, every rotated box would be flagged.
  - **A leaking run** is a thin crossing whose k + 1 faces are all open (θ > 0). Its evidence is its location and its open area: its smallest θ times dx².
  - **A part that closes no face** also leaks. The opposing-normals test cannot see a compact blob. A 2 dx sphere centred on a cell corner is crossed only by lines ρ = 0.71 R off its centre, where `n_in · n_out = 2ρ² − 1 = 0`, the same as a box's 90° edge. A blob has no closed face, so it is never "at an edge".
- **What a leaking part becomes** (L6: never silent):
  - **At an edge: announced.** Every leaking run lies within 4 cells (Chebyshev) of a cell with a face the part closes (θ = 0), where the part is resolved. Their total open area is at most 2% of the part's frontal area (the largest face of its bounds). The run line names the part, its number of leaking runs and their total open area.
    - Why 4 cells: a 30° wedge closes faces from about 2–4 dx from its tip, so its tip leaks lie within that reach.
    - A long thin plate has no closed face, so it is never "at an edge".
  - **Sheet-like: a sheet, announced.** Any other leaking part whose extents are plate-like (smallest < 2 dx ≤ middle) becomes a sheet with the smallest extent as its thickness. The run line names it, with its declared and measured thickness.
  - **Otherwise refused by name,** with the first leaking run's location and "declare min_feature or refine dx".
- **D5's 2 dx minimum is not sufficient for compact parts.** A sphere 2 dx across, centred on a cell corner, has eight centres inside it, and the faces between them have θ ≈ 0.21. It closes no face and is a porous blob. The safeguard catches it.
- **Tests** (P2's Task 2): a hub with a thin fin longer than 4 cells (the hub closes faces, the fin's leak is found, the part refused) and the same hub with a fin shorter than 4 cells (announced, with its open area: the boundary between the two, the lead); a 1.5 dx fin with centres inside it (found); a 2 dx sphere at its worst alignment (refused); a 4 dx box turned 45° (NOT flagged); a 30° wedge of 64-cell chord whose tip leaks (announced, with its area: 2% of the bounds' largest face admits a 30° edge only from about 60 cells of chord); a long plate 0.5 dx thick declared thick (sheeted, never "at an edge"); and the screen mutation `φ₁ + φ₂ < dx/2`, which must miss a crossing the test then catches.

## 5. Porous parts (Darcy–Forchheimer, confirmed 2026-10-12)

- **The sink, implicit with the pressure** (the lead's finding 1, amended 2026-10-13). A porous winner gives θ = 1, so the faces stay open.
  - **The factor.** Each face it covers takes `β = 1 / (1 + s h c)`, with `c = ν/K + ½ C₂ |u*|` and `s = clamp(0.5 − φ/dx, 0, 1)`, the fraction it covers. `|u*|` is the speed at the face before the projection, its other two components the mean of their four nearest samples.
  - **The solve.** The projection's face weight is `w = θ β`, and the update is `u = β (u* − (h/ρ)∇p)` (§3). That is the point-implicit solve of `∂u/∂t = −∇p/ρ − s c u` with the pressure, with c lagged only through `|u*|`. It is stable at any `h`.
  - **Why.** At a steady state `u = β (u − (h/ρ) ∂p)` gives `∂p = −ρ s c u`, the law's drop at any `h`. Split before the projection, as first written, the drop read low by `1/(1 + s h c)`: 17% for Test/Docs' `k` = 1 screen at `h` = 1 ms (S4).
  - The sink is no longer a step of its own.
- **The thresholds.** A porous part takes D5's 2-cell rule like a wall, but an under-resolved porous part becomes no sheet: it is refused. A thin mesh screen is better as a porous sheet with a pressure-jump law, which is a later case.

## 6. Wall pressure and force (P3)

- **On a cut cell.** The divergence theorem gives the solid surface's area vector inside cell `c` as `A_c = Σ_f θ_f dx² n_f`, summed over the cell's faces with `n_f` pointing out of the cell.
- **The force and moment** on the body: `F = −Σ_c p_c A_c` and `M = Σ_c (x_c − com) × (−p_c A_c)`. They are summed in the reduction's fixed tile order (`reduce.hpp`), so they are deterministic and match the GPU.
- **On a sheet,** both sides contribute: the pressure difference across each closed face, times `dx²`, along `e_face`.
- **For display,** the view reads `C_p` off the wall (Rendering's RND-8). It samples the pressure at `x + n·δ` along the outward normal, with δ about one cell. On a sheet that keeps each side's pressure, where the wall itself would average across the jump. Publish's first solid layer (P6) stays for continuity and for samples taken at the wall, but neither the `C_p` view nor P3's force and moment relies on it. P3 reads the cut cells' own pressures. One-way, F and M are display only; in D4 they replace the body's drag element (spec §2).
- **Publish next to solids and sheets** (Rendering, 2026-10-13):
  - Publish's gradient takes **one-sided differences at a closed face** (θ = 0), a wall's or a sheet's, as it already does at a non-periodic domain edge. A central difference across a sheet would mix the two sides' velocities and draw spurious vorticity, Q and λ₂ along a sheet that has none.
  - `PublishInput` gains the face apertures, or a per-face closed flag, beside the solid mask.
  - Test (§8 test 8, Test/Docs' S5): a field linear on each side of a sheet, with jumps in u and w across it. The published |ω| and q̂ next to the sheet match the far field's. A continuous shear could not tell the one-sided rule from a central difference.

## 7. P2b: the drone box's boundaries (demonstration plan, step 5)

One function maps a flight condition to the domain's boundary conditions and places the box:
- `hover`: every face open, `ambient` 0;
- `hover + wind w`: `ambient = w`, the floor wall still;
- `forward v`: `ambient = −v`, and the floor is a rolling road with `wall_velocity = −v`.

With a floor, the box's lower face sits `h` below the hub plane, and the drone stays centred horizontally. A height that leaves a part outside the box, or within 2 cells of the floor, is refused by name.

## 8. Tests, each red first

Every band, configuration and reading time comes from Test/Docs' note, `../../test-docs/2026-10-13-solids-tolerances.md` (S1–S5, fixed before the first run). No band is measured. Each record's `tolerance:` line cites the note. Where a check could pass on a wrong solver, a named mutation is run once against it during development and recorded in the ledger: the check must fail under the mutation (the lead's finding 4).

1. **A sphere in a uniform stream (S1)**: free slip, a box of side 8R, the centre on a cell corner, `U h/dx = 0.1`, `n_v = 16`, R/dx 6 (R/dx 12 opt-in).
   - The added-mass impulse `C_M`, from P3, at step 1.
   - `C_p` one cell out along the normal against potential flow at the same points, and the drag and side forces, at step 3.
   - The flux and divergence checks stay.
   - Each step's solve must reach a final residual of at most 10⁻⁴ of its starting one, and the record prints it (Test/Docs). An unconverged solve then shows as a miss rather than quietly biasing `C_M`.
   - d'Alembert's zero cannot fail on its own, since the cut cells' area vectors sum to zero. The added mass can. Mutations: a P3 that returns 0 (reads `C_M` = 0) and a P3 with its sign flipped (−0.5). Both must fail.
2. **Circular Couette flow (S2)**, which replaces the rotating cylinder in still air (confirmed: it has a closed form). The cylinder turns inside a fixed coaxial tube, `R₂ = 2R₁`, `Re` 10, quasi-2D.
   - `u_θ` against `A r + B/r`, with the order check from R₁/dx 16 to 32.
   - The design's two checks stay, as exact ones: the closed faces carry the rigid velocity bit for bit, and the air's circulation has the rotation's sign.
   - It exercises §3's viscous stencil at an immersed wall. Mutation: the rigid velocity at closed faces only (no crossing-link term), which Test/Docs expects to miss by up to a factor of two.
3. **A sheet across a periodic channel (S3)**, one step from a cold start, run with the sheet on an even and on an odd fine plane.
   - No flux, `max |u|/U`, the pressure step, P3's force, and the run line naming the sheet.
   - `n_v` = 32 counts CG iterations.
   - **The convergence check** (the lead's finding 2): from the cold start, both parities, the solve's residual falls to 10⁻³ of its start within 8 CG iterations. This is an implementation band under `TD-2`, not one of Test/Docs' tolerances: the port measured it at 4 iterations, and it is pinned at 8. The port reaches 10⁻⁸ in fp64, and fp32 floors near 10⁻⁶. Mutation: plain V-cycles forced on the odd plane, which the port leaves at 0.6.
4. **A porous block (S4)**, three Forchheimer numbers at `h c ≤ 10⁻³`, and the `h c = 1` case, which Test/Docs now checks at 1 ± 0.005: with §5's implicit sink the steady drop is exact at any `h`. Mutation: the sink split before the projection, which reads 0.5 at `h c = 1`.
5. **Refusals:**
   - a wire-like part (two extents below 2 `dx`), named with the `dx` it needs;
   - a floor height that cuts a part;
   - a closed region with a net prescribed flux. Example: a closed box whose −x and +x walls move inward and outward, cut across x by a sheet. The domain balances; each region does not.
6. **No solid, no change.** With no parts, both goldens are bit-identical.
7. **The posed drone**, from a compiled airframe (A4) and from a mesh part (B4) of the same box: the same apertures within the SDF's rounding (`TD-2`, measured and pinned, as Test/Docs' note says).
8. **Publish beside a sheet (S5)**: a field linear on each side, with jumps in u and w across the sheet. The published |ω| and q̂ next to the sheet match the far field's. Mutation: a central difference across the sheet (the closed-face flag ignored), which publishes |ω| ≈ 70 S.
9. **Fluid regions** (the lead's finding 3): a closed box cut by a sheet into two closed regions of unequal size.
   - Each region's mean p is 0 to fp32 rounding.
   - The run line reports "2 closed".
   - The divergence keeps plan A's band in each region.
   - Mutation: one mean removed over all the fluid, which leaves each region's mean wherever the solve put it.
10. **The preconditioner is symmetric** (the lead's condition): `⟨Mx, y⟩ = ⟨x, My⟩` to rounding for random mean-free vectors on S3's grid with its sheet, since CG needs it. Mutation: plan A's restriction (the children's sum) in place of full weighting. The port measured 10⁻¹ of asymmetry with it.
11. **The crossing fraction's floor** (the lead's condition): a wall a hair from a velocity sample, `φ_self` = 10⁻⁶ `dx`. The link takes f = 0.01, so the sample's diagonal is at most `1 + (5 + 100) α`, and the solve stays finite and within the wall's velocity bounds. Mutation: no clamp, which gives `f ≈ 10⁻⁶` and a diagonal near `10⁶ α`.

Test 1's surface `C_p` is the solver's half of Rendering's R2.5. Rendering's half compares the view's sampled `C_p` with P3's at the same surface points.

## 9. Order and cost

P2 (walls, sheets, the posed drone, the solver with solids, the fluid regions, the viscous wall stencil; tests 1–3, 5–7 and 9), then P3 (force and moment, `C_p`; test 8), then porous (test 4), then P2b. Test 1's added mass and test 3's force read P3, so their checks join P3. Their P2 checks (flux, divergence, the closed faces' velocities, convergence) come first.

CPU cost: about 300 k samples per rebuild at a 42³ support box. Since the parts are rebuilt only when the attitude changes, a held attitude costs nothing per step. The projection's θ path is the general stencil's cost.

## 10. Decisions (the lead, 2026-10-13)

1. **Inside a solid, a backtrace reads the owning part's rigid velocity by default.** That is exact for a no-slip moving wall, and it fits Cameron's accuracy bar. Free-slip extrapolation, two layers by fixed sweeps, stays a named option in the configuration. The spec's §2 is amended to match.
2. **Sheets are binary and staircased in this first version.** That is a **known first-order limitation**: a sheet's surface is resolved only to the cell, and its closed faces are either fully closed or fully open. Exact cut-face sheets, with crossing fractions, come later, when V2's flat plate (drag against `Re`) measures the difference.

## 11. Amendments (2026-10-13): Test/Docs' tolerances and the lead's five findings

Test/Docs fixed the tests' tolerances before any run (`../../test-docs/2026-10-13-solids-tolerances.md`, merged at 5eb7ea7). The lead made its five findings binding. Each is answered in place:

1. **Porous splitting:** the sink moves into the projection as the face factor β (§5). The steady drop is then exact at any `h`, and S4 pins it.
2. **Sheets the coarse levels cannot see:** with solids, the projection runs CG preconditioned by a symmetric V-cycle (§3), and test 3 gains a convergence check on both parities. Measured on a port of plan A's cycle:
   - plain cycles stall on odd-plane sheets;
   - a min or a series coarse rule diverges;
   - MGPCG converges on both parities.
   **This reverses spec §1.5's rejection of CG for the first cut, and needs the lead's approval.**
3. **Separate fluid regions:** regions are labelled deterministically. Each closed region has its own mean removed, and a closed region with a net prescribed flux is refused by name (§3). The run line reports the count. Tests 5 and 9.
4. **Checks that could not fail:** test 1 reads the added mass, and test 8 has jumps across the sheet (Test/Docs' S1 and S5). Every such check has a named mutation, run once and recorded (§8).
5. **The viscous stencil at a wall:** a symmetric ghost-fluid Dirichlet condition, with the rigid velocity on φ = 0, the surface the apertures put at θ = ½ (§3). Test 2, now circular Couette flow (S2), exercises it.
6. **The measured safeguard** (the lead's ruling, 2026-10-13, after Physics found the volume comparison unbiased and so blind): thin by `min(min_feature, smallest bounds extent) < 2 dx` through one `classify_part()`; leaking runs found by a Lipschitz screen, sphere-traced entry and exit, and opposing normals; edge leaks announced within 4 cells of the part's closed faces and 2% of its frontal area; sheet-like leakers become announced sheets; the rest are refused (§4).
7. **Inside a solid and the layering** (approved by the lead, 2026-10-13): closed samples carry the rigid velocity, and the free-slip option fills two Jacobi layers; the airflow core takes its solids through a sampler callback and per-part data, never including `vehicles/` (§3). The floor-height refusal belongs to P2b (§7).
