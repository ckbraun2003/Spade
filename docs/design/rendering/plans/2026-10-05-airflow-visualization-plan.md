# Airflow visualization plan

**Owner:** Rendering. **Status:** approved by the lead, 2026-10-05; steps 1–5 started. The lead's answers:
- **Q1:** Core's call, asked directly; `core/` if Core agrees.
- **Q2:** asked of Physics as a spec amendment; the lead supports it.
- **Q3:** ray-cast iso-surfaces, yes.
- **Q4:** arc-length RK4 at a fixed quarter cell, yes; adaptive steps can come later as an opt-in.
- **Q5:** fixed ranges by default, yes; the auto range is an explicit opt-in that announces the range it chose (`L6`).
- **λ₂:** out for now. A follow-up is recorded in `07-status`: a symmetric 3×3 eigen-solve by fixed-sweep cyclic Jacobi needs only `√`, so it is `TD-3`-clean and deterministic.

**Inputs:**
- The joint spec `../../physics/plans/2026-10-05-airflow-design.md`: §5 (Rendering), §6 (the views and live-smoke checks Interface needs), §11 (`PHY-9`: "F1 level, NASA level aerodynamics").
- Physics' plan A, `../../physics/plans/2026-10-05-airflow-solver-core-plan.md` (the CPU MAC grid and V1).
- The lead's scope, 2026-10-05: volume rendering with transfer functions; RK4 streamlines and smoke; vorticity and Q-criterion iso-surfaces; the field channel. Fed from plan A's CPU grid first, so every view is tested before the GPU twin exists.

## Summary

Four views of one published flow volume, each built on the CPU first as the reference (`RND-3`), then on GL, the editor's path:
1. **Volume rendering** of speed, pressure and vorticity, through transfer functions, with dye as smoke.
2. **Iso-surfaces** of vorticity magnitude and the Q-criterion, ray-cast in the same march.
3. **Streamlines** by classic RK4, and **tracer particles** for streaklines.
4. **The field channel:** slices through the volume as field layers, and any field coloured onto solid surfaces (the surface-pressure view).

What makes this research grade, not decoration:
- Every view is checked against an analytic answer, never only against itself (`TD-4`).
- Transfer functions are physical and published: Beer–Lambert absorption, perceptually uniform colour maps with provenance, and fixed ranges for comparisons.
- The integrator's order is measured.

## 1. The input: one flow volume

The views read only Publish (`L5`). The spec fixes the format (§5): a header plus two textures, cell-centred.
- **V**, RGBA16F = (`u, v, w`, dye).
- **S**, RG16F = (pressure gauge, vorticity magnitude).
- **The header:** the world box (origin, cell size, dims) and a finite min and max per channel.
- **Conversion:** f32 to f16 rounds to nearest even, keeping subnormals. A non-finite sample is no-data and draws transparent.

This plan asks for two changes to it (Q1, Q2):
- **A finite value past the f16 range (65 504) is no-data too,** counted in the header, never saturated: a clamp would draw the wrong value in silence (`L6`).
- **S gains a third channel, `q̂ = sign(Q) √|Q|`**, in 1/s like vorticity, computed in fp32 by Publish from the MAC grid's own staggered derivatives. Iso-surfaces need Q accurate to well under its iso-value. Rebuilding Q in the renderer from f16 velocities would difference values that agree to three digits. Q itself overflows f16 inside a rotor's tip vortex (|ω| ≈ 10³ 1/s gives Q ≈ ¼|ω|² ≈ 2.5·10⁵), and `q̂` does not. With `√` only, it stays inside `TD-3`.
- **Cost:** S becomes RGBA16F with one channel reserved (zero), so 16 B/cell, 32 MiB at 128³. That is Physics' VRAM table to amend.

**Derived on the CPU reference, in fp32:**
- **Speed** is `|uvw|`.
- **Sampling** is trilinear, cell-centred and clamped at the box's edges. f16 decodes exactly.
- **A gradient,** for iso-surface normals, is the central difference of the trilinear field at a step of one cell.

## 2. Volume rendering (emission and absorption)

**The march (CPU reference, `render/flow_volume`).**
- **Geometry:** per pixel, the camera ray is clipped to the volume's box and to the opaque depth already in the frame, as `draw_csg_subtrees` clips today. The march then steps front to back at a fixed half cell.
- **Compositing:** each step adds emission `c·α` weighted by the transmittance so far, then multiplies the transmittance by `exp32(−σ·Δs)`, Beer–Lambert with `core/fp32_math`'s `exp32`. It exits early when the transmittance falls under 1/256.
- **The last partial step** takes its true length, so a frame does not depend on where the box starts.
- **Composite:** the result goes over the opaque frame: `C = C_vol + T·C_behind`.

**Transfer functions (`render::TransferFunction`).**
- **The colour map:** one per channel, over a range.
- **The extinction:** `σ(value)` in 1/m, piecewise linear in up to 8 points, so a band of interest can be made dense and the rest clear.
- **The range:** fixed per scene by default, because comparing frames or cases needs the same mapping. The header's min and max is a toggle, for exploring.
- **Colour maps, tabulated at 256 entries, committed with provenance:**
  - speed: *viridis*, from matplotlib's CC0 data;
  - pressure: *cool-warm*, Moreland's diverging map (2009), centred on zero gauge;
  - vorticity and dye: *inferno*.
  The field layer's colour map grows into this type, so slices and volume share one mapping.

**Smoke (dye).** Absorption plus one constant in-scatter colour (the spec's amendment). No self-shadowing at first.

**What the checks test (red first, each):**
- A uniform slab of known `σ` and thickness transmits `exp(−σL)` within one f16 step.
- A step at a different box offset gives the same pixel.
- Opaque geometry inside the box hides the air behind it, both ways (as `DepthOrdersCsgAgainstMeshesBothWays`).
- A no-data cell draws as clear and is counted in the frame's report (`L6`).
- **Golden:** a synthetic vortex ring with a Gaussian dye blob at 32³ (the spec's choice), pending `TD-12` until gcc reproduces it.

## 3. Iso-surfaces: vorticity and the Q-criterion

**Ray-cast, not meshed.**
- **Finding the hit:** within the same march, a sign change of `f − f_iso` between two steps is refined by four bisections on the trilinear field. The hit is depth-tested like any surface.
- **Normal:** the field's gradient.
- **Shading:** the scene's sun with `shade_vertex_color()` (one shading model).
- **Up to four iso-values,** each opaque or translucent. The colour can be fixed, or another channel at the hit: the standard "Q iso-surface coloured by speed".

**Why not marching cubes:**
- A mesh needs a per-frame extraction pass, on two backends, with its own goldens and its own fold rules.
- The ray-cast is per pixel, so it agrees across backends to a band.
- A mesh comes later only if export to another tool is asked for.

**Fields:**
- **`|ω|`** from S.
- **The Q-criterion** through `q̂` (Q2): an iso-surface `Q = Q₀` is `q̂ = √Q₀`, with no division and no transcendental.
- **λ₂** is left out: it needs a symmetric 3×3 eigen-solve, whose closed form uses `acos` and cube roots, outside `TD-3`. Q is the standard for vortex identification in these flows.

**Checks, against analytic fields:**
- **Lamb–Oseen vortex:** Q and `|ω|` are closed-form functions of radius, so each iso-surface is a cylinder of known radius. Rendered from the axis's side, its silhouette width must match the analytic cylinder within one pixel plus the trilinear error, both stated.
- **Taylor–Green vortex at t = 0** (plan A's C0 initial field): Q is closed form. A ray-cast of the analytic Q (fp64, test-only) is the reference, and agreement is banded like `RS4`.
- **Golden:** the vortex ring's Q iso coloured by speed.

## 4. Streamlines and smoke tracers

**The integrator (`render/flow_lines`).**
- **Scheme:** classic RK4 in fp32 on the trilinear velocity, in arc length: `dx/ds = u/|u|` at a fixed `Δs` of a quarter cell, so polyline points are evenly spaced.
- **Direction:** forward and backward from each seed.
- **Fixed step, never adaptive:** an adaptive step makes the two backends take different step counts. For the same reason the spec refuses adaptive CFL (§8 Q2).
- **Termination, recorded per line (`L6`):** leaving the box; speed under a stated floor (stagnation); a no-data cell; the step budget.

**Seeds:** a rake (a line of `n`), a grid on a plane, or rings around each rotor hub (`R` and hub positions from the scene). Seeds are inputs; nothing is random.

**Drawing.** Polylines through the raster's existing line path, depth-tested, coloured through the speed transfer function. Tubes and ribbons come later.

**Tracers (streaklines, the smoke the eye knows).**
- **Advection:** particles move through the time-varying published field by the same RK4, in time, at the frame's `dt`. They are emitted at fixed sources on a fixed schedule, with no RNG.
- **State:** the particle state is an explicit object the caller owns and passes in and out. The technique stays a pure function of its inputs (`01-techniques`).
- **Drawing:** screen-space points, depth-tested.

**Checks:**
- **Uniform flow:** straight lines, exact.
- **Solid-body rotation:** circles. After one revolution the closure error is measured at `Δs` and `Δs/2`, and the ratio must show fourth order (≈ 16, with the trilinear field's own error stated).
- **Interpolation:** the trilinear field reproduces any linear velocity field exactly.
- **Rotor seeds:** seeds 0.5R below a hub move down faster than half of `v_h` (the live smoke's `airflow.streamlines`, §6), on plan A's actuator-disc field.
- **Tracers:** a tracer in uniform flow lands at `x₀ + u·t` exactly.

## 5. The field channel

- **Slices:** a `FieldLayer` filled by sampling a volume channel on its cells (trilinear), so the existing CPU and GL slice view reads airflow with no new drawing code. This is the spec's kept 2D view (§5), and part of the field-channel plan's Part B.
- **Surface colouring:** each opaque pixel inside the box takes a channel's value at its world position, reconstructed from depth, through that channel's transfer function. Gauge pressure on the vehicle's surfaces is the surface `C_p` view an aerodynamicist reads first. It is a shaded-mode option and changes no geometry.
- **Checks:** a slice through a linear field equals the field. Surface colouring on a plane in a linear pressure field gives the analytic gradient across the screen.

## 6. Feeding from plan A's CPU grid

The views take only the flow volume, never `MacGrid`, so rendering cannot depend on the solver (`L5`; plan A's layering keeps `physics/airflow/` free of `render/`). The conversion is Publish's:
- interpolate the faces to cell centres;
- compute `|ω|` and `q̂` from staggered differences;
- convert to f16.

It belongs to Physics. Until plan C's module exists it can be a pure function beside plan A, `publish_render_volume(const MacGrid&, …)` (Q1 says where the format type lives).

The integration tests then run plan A's real fields through every view:
- **The Taylor–Green vortex:** Q at t = 0 is analytic (§3), and it decays on record from C0.
- **The actuator disc's wake:** the streamline check in §4, and the volume and iso views as smoke tests.

## 7. GL, the editor's path

After the CPU reference:
- **The volume and iso pass:** the volume uploads to two 3D textures. A box back-face proxy with depth clamp is drawn, as B2's GL pass does, and the fragment shader ports §2 and §3. It reads opaque depth from an internal texture redrawn depth-only (§5's amendment).
- **Sampling:** manual trilinear by `texelFetch`, not hardware filtering, so the band against the CPU stays tight.
- **Streamlines** draw as `GL_LINES`, tracers as `GL_POINTS`. Integration stays on the CPU until seed counts need a GL compute port, banded against the CPU.
- **Bands:** measured on the RTX 3060 Ti, with the device and driver recorded (`TD-13`).
- **Cost:** at 128³ and 1080p, measured against §5's 2–4 ms estimate.
- **The GL CSG march's uniform-buffer follow-up** (`07-status` debt) rides with this pass, since both draw a box proxy.

**The Vulkan raster needs nothing** (spec §5); when it resumes, this is a compute pass over the published buffer.

## 8. Steps

Each step is red then green, gcc-checked, and reviewed. Goldens wait for `TD-12`.
1. **The flow-volume format and the synthetic volumes:** the format (where Q1 puts it), f16 conversion with the no-data rules, and the analytic generators (vortex ring, Lamb–Oseen, Taylor–Green, uniform flow, solid-body rotation, dye blob), test-side.
2. **The CPU volume march and the transfer functions,** with the colour-map tables and their provenance. Slab, offset, depth and no-data checks; the vortex-ring golden.
3. **CPU iso-surfaces** (`|ω|`, `q̂`): the Lamb–Oseen and Taylor–Green checks; the Q golden.
4. **CPU streamlines and tracers:** the RK4 order check, termination reasons, line drawing, and the streamline golden.
5. **The field channel:** slices from the volume, and surface colouring.
6. **Plan A's feed:** once Physics' `publish_render_volume` and plan A's Task 8 land, the Taylor–Green and actuator-disc integration tests.
7. **GL:** the volume, iso and line passes; bands on the 3060 Ti; the cost at 128³.
8. **Docs:** `01-techniques` (a flow-volume technique and its grade), `07-status`, the spec's §5 amendments as landed.

Steps 1–5 need nothing from Physics or Core beyond Q1. Step 6 waits on plan A, step 7 on nothing.

## 9. Questions for the lead

- **Q1. Where the flow-volume type lives.**
  - (A) `core/`, so Physics' Publish and Rendering both include it. That is Core's directory, and Core's say.
  - (B) `render/`, with the MacGrid conversion in tests only until plan C, when Core's Publish machinery places it.
  - **Recommend A:** one type from the start, and no test-only copy of a production conversion.
- **Q2. `q̂` in S** (`sign(Q) √|Q|`, so S becomes RGBA16F with one channel reserved, 16 B/cell).
  - (A) Yes: Physics computes it in fp32 from the MAC grid.
  - (B) No: Rendering derives Q from f16 velocity.
  - **Recommend A.** B loses the three digits iso-surfaces need, and Q overflows f16 in tip vortices. It is a spec amendment, so it is Physics' to fold, with the VRAM table.
- **Q3. Iso-surfaces.** (A) Ray-cast in the march. (B) Marching-cubes meshes. **Recommend A;** B only if export is asked for.
- **Q4. Streamline parameter.** (A) Arc length at a fixed quarter cell. (B) Pseudo-time at a fixed `h`. Adaptive RK45 is refused, for determinism and parity. **Recommend A:** evenly spaced points, and step counts independent of speed.
- **Q5. Transfer-function ranges.** (A) Fixed per scene by default, the header's range as a toggle. (B) The header's range by default. **Recommend A:** a research comparison needs the same mapping across frames.

Out of scope here:
- λ₂ (§3);
- volume lighting and self-shadowing;
- line integral convolution on slices;
- pathlines over recorded runs.
Each is a later item if asked for.
