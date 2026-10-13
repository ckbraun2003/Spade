# The C_p surface view on the solid drone (R2) — plan

The drone demonstration's R2 (`../../physics/plans/2026-10-12-airflow-drone-demonstration-plan.md`): the drone's surfaces coloured by the pressure coefficient from the solved wall pressure, in D1's offline CPU frames and D2's live GL view. It waits on Physics' P2 (immersed solids) and P3 (wall pressure). This plan readies it, so the work starts when P2 lands.

## What exists

The airflow visualization plan's step 5 and step 7 (merged):
- `FlowSurfaceColouring` (`render/flow_field.hpp`): each opaque pixel inside the volume's box takes a quantity's value at its world position, reconstructed from depth, through a transfer function. It is unlit, and a no-data sample keeps its shaded colour and is counted.
- Both paths draw it. GL is banded against the CPU (1 level at worst, 861 pixels differ).
- Physics' P6 (in Publish): the first layer of solid cells takes a fluid-side extrapolation, and deeper solid cells are no-data.
- Publish's pressure is gauge pressure in Pa. The open faces are pressure outlets with p = 0 (`physics/airflow/boundaries.hpp`), so 0 is the ambient.

## The gaps

1. **Units.** The view shows Pa, and R2 asks for C_p = (p − p_ref) / q_ref. Hover has no freestream, so q_ref must be chosen there (Q1).
2. **Where the wall pressure is read.** A sample at the surface point interpolates across the wall, between fluid cells and P6's extrapolated layer. Three things follow:
   - A part two or three cells thick (the arms are about 2.5 cells at D1's 6 mm nest) has first-layer cells whose fluid-side mean averages its top and bottom. The view would smear exactly the pressure difference that makes lift.
   - At edges and corners, a stencil can reach a deeper solid cell, which is no-data, and the pixel loses its colour.
   - The value depends on P6's extrapolation rule, which is Publish's choice, not a measured wall pressure.
3. **What takes the colour.** Every opaque pixel in the box is coloured. In D1 the rotors are actuator models, not walls in the flow, so a drawn propeller disc or blade would show a "surface pressure" the solver never had. The floor is coloured too. That is informative under a rotor in ground effect, but it must be chosen, not accidental.
4. **No check against the solver.** P3 computes the wall pressure at body surface points, and the view never compares against it.

## Design

**R2.1, C_p as a transfer function.** Add a render helper `cp_colours(q_ref, p_ref, cp_min, cp_max, map)`. It returns a pressure `TransferFunction` whose fixed range is `[p_ref + cp_min·q_ref, p_ref + cp_max·q_ref]`. The views gain no new quantity and no shader change. Interface labels the legend in C_p from the same four numbers. `p_ref` defaults to 0, the outlets' ambient.

**R2.2, read the pressure off the wall.** Add `FlowSurfaceColouring::offset_cells` (default 0, today's behaviour). The C_p view sets it, and the sample is taken at `x + n·δ`, where n is the surface's outward normal and δ is offset_cells volume cells. This is standard wall post-processing: the pressure varies little across a boundary layer (∂p/∂n ≈ 0). The sample then reads fluid cells, not the extrapolated layer, so both sides of a thin part keep their own pressure.
- **The normal.** GL has it in the mesh and CSG fragment shaders. The CPU raster gains a normal buffer, allocated only when a colouring asks for an offset and written by the mesh and CSG passes beside depth.
- **δ is measured, not assumed** (R2.4): the candidates are 0.5, 1 and 1.5 cells. On a curved wall ∂p/∂n = ρu_t²/R, so the offset costs about 2(δ·dx/R)(u_t/U)² in C_p. That is large on a part only a few cells across.
- **Resolution.** C_p is honest only where a part spans several cells. The view reports, and Interface shows (L6), the smallest feature under colour in cells. Below a threshold set from R2.4, it is labelled "under-resolved".

**R2.3, only flow walls take the colour.** Add a per-draw-item flag (`DrawItem` and `CsgSubtree` through their draw item): "a wall in the flow". The colouring applies only to pixels whose nearest surface carries it. The sandbox sets it from `CORE-7`'s parts: wall parts yes; rotors under an actuator model no; porous parts shown as their own option. The floor's ground plane takes a scene flag of its own.

**R2.5, check against the solver.** Once P2 and P3 land, a feed test, as step 6 did for Taylor–Green and the disc:
- a published flow past a sphere at moderate Re, on the solver's grid;
- the front stagnation point within a measured band of C_p = 1;
- the attached front against potential flow, C_p = 1 − (9/4) sin²θ, up to the separation angle;
- the view's sampled C_p at P3's surface points against P3's own wall pressure, within a measured band. The view and the solver's force must tell one story.

## Tests (red first, each with a mutation)

- `cp_colours`: the range maps C_p's ends, and q_ref 0 or a non-finite value is refused by name.
- **Offset on a synthetic sphere (R2.4).** Potential-flow pressure around a sphere, written through the publish format with P6's first-layer rule. Render it with δ = 0, 0.5, 1 and 1.5 at R/dx = 4, 8 and 16, and compare each pixel's C_p with the analytic C_p at its surface point. The measurements pick δ and the "under-resolved" threshold, and the chosen δ's band is pinned.
- **The thin plate.** A two-cell plate with different pressures above and below. At δ = 0 the two faces read the same mean; at the chosen δ, each reads its own side. This is the test that justifies R2.2.
- **Walls only.** A scene with a flagged and an unflagged object in the box. Only the flagged one changes colour.
- **GL against the CPU** for the offset, as the step 5 band.
- **The feed test (R2.5)**, after P2 and P3.

## Rulings (the lead, 2026-10-13; `RND-8`)

- **Q1: A, with one rule.** In hover the reference is the tip's ½ρ(ΩR)², labelled "C_p (tip)". Render does not choose it. `cp_colours` takes `q_ref` and `p_ref` as inputs, and the host passes Interface's `spade::analysis` `reference_speed()` (tip for rotorcraft, freestream for a propeller; Cameron's Q6). The legend names which (`TD-9`).
- **Q2: a scene flag, off by default.**
- **Q3: yes.** R2.1, R2.3 and R2.4's synthetic work start now.

## Questions for the lead (as asked)

- **Q1. Hover's q_ref.** With wind, q_ref = ½ρV². In hover there is no freestream:
  - (A) tip speed, ½ρ(ΩR)², the rotor convention, labelled "C_p (tip)";
  - (B) the induced velocity v_h;
  - (C) pressure in Pa in hover, with C_p only with wind.
  - **I recommend A**, since it is what rotor aerodynamics reads.
- **Q2. The floor.** Colour the ground plane by pressure in ground effect (a scene flag, off by default), or never? **I recommend the flag**, off by default.
- **Q3. Order.** R2.1, R2.3 and R2.4's synthetic tests do not need P2. Start them now, or wait for P2 to fix the geometry? **I recommend starting now**, so R2.5 is the only step left when P2 and P3 land.

## Dependencies and order

1. Now: R2.1, R2.3, then R2.2 with R2.4's measurements (CPU normal buffer, GL offset), each red then green.
2. After P2 and P3: the R2.5 feed test.
3. D1: Interface's I1 sets the colouring per frame (the walls flag from the posed parts, and q_ref from the flight state), and the airflow tour checks that C_p is finite.
4. Docs: `07-status`, plus this plan's measured δ and threshold.

A stale comment found on the way: `RenderScene::flow_surface_colouring` still says "GL refuses it until its pass lands", and step 7 landed it. The fix rides on R2.1's branch.
