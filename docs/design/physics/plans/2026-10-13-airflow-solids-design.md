# Solids in the airflow grid: P2, P2b and P3 — design

**Owner:** Physics. **Status:** **APPROVED by the lead, 2026-10-13**, with §10's two decisions. It implements the demonstration plan's P2 (immersed solids), P2b (the drone box's boundaries) and P3 (wall pressure) on plan A's CPU reference. The GPU twin follows in plan B's kernels.
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
- **The projection** solves `∇·(θ∇p) = (ρ/h) ∇·(θ u* + (1 − θ) u_s)`:
  - each face's weight θ multiplies its neighbour term in the stencil and in the diagonal;
  - `u_s` is the owner's wall velocity component normal to the face;
  - after the solve, a face with θ > 0 takes `u* − (h/ρ)∇p`, and a face with θ = 0 takes `u_s`.
- **Bit identity.** With no solid the stencil is plan A's, bit for bit: `1.0f * p` is `p` and the six weights sum to exactly 6. So both goldens hold, and the solids path is taken only where a θ < 1 exists.
- **The multigrid.** Each level holds its faces' θ. A coarse face's θ is the mean of the fine faces it covers, which the spec states. A cell whose faces are all closed has diagonal 0: the smoother already skips it (`smooth_one_cell`), and its residual is 0.
- **A closed domain with solids.** The mean is removed over the cells the fluid reaches (diagonal > 0), not over every cell, so the compatibility condition holds for the fluid region. The drone box has outlets, so this applies only to validation cases.
- **Inside a solid.** Backtraces that land in a solid read the owner's rigid velocity there. A wall face is a no-slip, moving wall, and the rigid field is exact inside it. The spec's free-slip lean mode extrapolates instead, two layers by fixed sweeps; it stays an option for coarse grids, named in the configuration.
- **The minimum resolution (D5).** A wall part's `min_feature` must be at least 2 `dx`. Section 4 says what happens below that.

## 4. Thin walls: labelled sheets (the lead, 2026-10-12)

The drone's frame plates and boards (1.5–4 mm) are thinner than 2 cells at D1's 6 mm nest.

- **The rule.** A wall part whose `min_feature` is below 2 `dx`, while its other two extents are at least 2 `dx`, becomes a **sheet**: zero thickness, at its mid-surface. In practice that is a plate, a board or a fin.
- **Which faces it closes.** A face is closed (θ = 0) when the sheet crosses the segment between the face's two cell centres: φ at the face centre is below `dx/2 · |n · e_face|`, with n the distance gradient. Otherwise the face is open. So a sheet is impermeable to first order, with a staircase surface. It cannot leak at half-open faces the way a sub-cell solid on the ramp would.
- **How it is labelled.** The run line and the records name each sheet part and its true thickness, for example "frame_top: sheet (2.0 mm < 2 dx = 12 mm)". Its pressure is integrated on both sides (§6).
- **What is refused by name.** A part below 2 `dx` in more than one extent, such as a wire or a small fastener, is refused with the `dx` it needs. It is never porous and never dropped. Interface's panel shows the refusal and the grid that would resolve the part.
- These thresholds are first values. Validation revises them: a flat plate normal to the flow, compared with the published drag against `Re`, is V2's case.

## 5. Porous parts (Darcy–Forchheimer, confirmed 2026-10-12)

- **The sink.** A porous winner gives θ = 1, so the faces stay open, plus the sink `u ← u / (1 + h (ν/K + ½ C₂ |u|))` on the faces it covers. The fraction it covers is `s = clamp(0.5 − φ/dx, 0, 1)`, and the sink is applied as `u ← u / (1 + s h (…))`. It is point-implicit, so it is stable at any `h`. It is applied after advection and before the projection, like the sources.
- **The thresholds.** A porous part takes D5's 2-cell rule like a wall, but an under-resolved porous part becomes no sheet: it is refused. A thin mesh screen is better as a porous sheet with a pressure-jump law, which is a later case.

## 6. Wall pressure and force (P3)

- **On a cut cell.** The divergence theorem gives the solid surface's area vector inside cell `c` as `A_c = Σ_f θ_f dx² n_f`, summed over the cell's faces with `n_f` pointing out of the cell.
- **The force and moment** on the body: `F = −Σ_c p_c A_c` and `M = Σ_c (x_c − com) × (−p_c A_c)`. They are summed in the reduction's fixed tile order (`reduce.hpp`), so they are deterministic and match the GPU.
- **On a sheet,** both sides contribute: the pressure difference across each closed face, times `dx²`, along `e_face`.
- **For display,** the view reads `C_p` off the wall (Rendering's RND-8). It samples the pressure at `x + n·δ` along the outward normal, with δ about one cell. On a sheet that keeps each side's pressure, where the wall itself would average across the jump. Publish's first solid layer (P6) still serves samples taken at the wall. One-way, F and M are display only; in D4 they replace the body's drag element (spec §2).
- **Publish next to solids and sheets** (Rendering, 2026-10-13):
  - Publish's gradient takes **one-sided differences at a closed face** (θ = 0), as it already does at a non-periodic domain edge. A central difference across a sheet would mix the two sides' velocities and draw spurious vorticity, Q and λ₂ along a sheet that has none.
  - `PublishInput` gains the face apertures, or a per-face closed flag, beside the solid mask.
  - Test: a sheet across a uniform shear, with no vortex on either side. The published |ω|, Q and λ₂ next to the sheet match the far field's.

## 7. P2b: the drone box's boundaries (demonstration plan, step 5)

One function maps a flight condition to the domain's boundary conditions and places the box:
- `hover`: every face open, `ambient` 0;
- `hover + wind w`: `ambient = w`, the floor wall still;
- `forward v`: `ambient = −v`, and the floor is a rolling road with `wall_velocity = −v`.

With a floor, the box's lower face sits `h` below the hub plane, and the drone stays centred horizontally. A height that leaves a part outside the box, or within 2 cells of the floor, is refused by name.

## 8. Tests, each red first

1. **A sphere in a uniform stream (free slip).** The projected field puts no net flux through the sphere's faces, and its divergence falls as plan A's does. Against potential flow (`1 − 9/4 sin²ϑ`), the surface `C_p` matches within a band measured at 12 cells per diameter. The net force is near 0 (d'Alembert).
2. **A rotating cylinder in still air (no slip).** Faces near the surface take the rigid velocity, and the air gains circulation in the rotation's sense.
3. **A sheet across a channel.** It carries no flux, and the pressure jumps across it. The run line names it a sheet.
4. **A porous block.** The pressure drop across it matches `L (ν/K u + ½ C₂ u²) ρ` within a band.
5. **Refusals:**
   - a wire-like part (two extents below 2 `dx`), named with the `dx` it needs;
   - a floor height that cuts a part.
6. **No solid, no change.** With no parts, both goldens are bit-identical.
7. **The posed drone**, from a compiled airframe (A4) and from a mesh part (B4) of the same box: the same apertures within the SDF's rounding.
8. **Publish beside a sheet** (§6): a sheet across a uniform shear publishes the far field's |ω|, Q and λ₂ next to it, with no spurious vortex sheet.

Test 1's surface `C_p` is the solver's half of Rendering's R2.5. Rendering's half compares the view's sampled `C_p` with P3's at the same surface points.

## 9. Order and cost

P2 (walls, sheets, the posed drone, tests 1–3, 5–7), then P3 (force and moment, `C_p`), then porous (test 4), then P2b.

CPU cost: about 300 k samples per rebuild at a 42³ support box. Since the parts are rebuilt only when the attitude changes, a held attitude costs nothing per step. The projection's θ path is the general stencil's cost.

## 10. Decisions (the lead, 2026-10-13)

1. **Inside a solid, a backtrace reads the owning part's rigid velocity by default.** That is exact for a no-slip moving wall, and it fits Cameron's accuracy bar. Free-slip extrapolation, two layers by fixed sweeps, stays a named option in the configuration. The spec's §2 is amended to match.
2. **Sheets are binary and staircased in this first version.** That is a **known first-order limitation**: a sheet's surface is resolved only to the cell, and its closed faces are either fully closed or fully open. Exact cut-face sheets, with crossing fractions, come later, when V2's flat plate (drag against `Re`) measures the difference.
