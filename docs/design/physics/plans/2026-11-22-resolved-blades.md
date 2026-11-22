# Resolved blades on the FPV: a plan (O184)

Status: PLAN, not code. Written by Physics, 2026-11-22 (box clock, `date -u` at each stamp below). Inputs: Core's section (data layout, determinism, pins), Rendering's section (what each phase shows, painting cost). Grades used: **measured** (a number read from a run or a file), **read** (read from the code), **derived** (arithmetic on those), **assumed** (a stated guess).

## 0. The ask and the short answer

Cameron: "truly resolving blades needs finer detail than your card can hold": "If we're hitting optimization roadblocks we need to make the roads." The card is an RTX 3060 Ti, 8,192 MiB, 1,005 MiB used by the desktop: **7.0 GiB usable** (*measured*). The engine today costs **221 B a cell** from my measured run (3,569 MiB at 16.96 M cells) and **234.6 B a cell** in the device runner's own figure (tests/d1_case.hpp:95), so the card holds 32 to 34 M cells at best (*measured*, *read*).

The plan makes the engine fit the problem in four moves, in this order, each shown on screen:

1. **Memory diet** (Phase 0): 221 B a cell to a target of about 85 B, so the card holds about 2.6 times the cells. First payoff, on screen: the O158 film box goes much deeper (§6).
2. **Local refinement** (Phase 1): fine cells only round the rotors, coarse elsewhere, 2:1 levels, conservative at the patch faces.
3. **Moving solids** (Phase 2): the blade as a cut-cell solid moving through a static fine patch, its wall velocity in the pressure solve (the engine's solid field already carries both the open fractions and a wall velocity, so this extends what exists).
4. **Validation and the film** (Phases 3 and 4): one resolved rotor against UIUC's measured point, then four rotors.

Total: **about 45 to 58 Physics working days, plus Core 8 to 11 and Rendering about 18 in parallel; about 55 to 95 GPU hours** (§9). **First milestone that shows something real:** the memory diet's deeper O158 wake, at about day 8 of the plan (§9), then **one resolved rotor in a nested patch, validated**, at about day 35.

## 1. Memory per cell: where the 221 B go

**Read from the code** (engine/physics/airflow/fluid_step.cpp `fluid_bytes`, solids.cpp, regions.cpp). A MAC grid has about 3 faces a cell, so in floats a cell:

| Item | floats a cell | bytes | what it is for |
|---|---|---|---|
| u, v, w, p | 4 | 16 | the state |
| `next` (advection's output) | 4 | 16 | double buffer |
| advection `hat`, `lo`, `hi` | 9 | 36 | the limiter's three face fields |
| multigrid p, b, r, every level | 3.4 | 14 | the pressure solve (levels 1/8 each) |
| divergence | 1 | 4 | |
| viscous right side | 1 | 4 | |
| u/v right sides on the pool | 2 | 8 | |
| du, dv, dw (increments) | 3 | 12 | the line's and discs' force |
| **solver subtotal** | **~27.4** | **~110** | `fluid_bytes()` (*read*, *derived*) |
| solid field: per face theta, wall velocity, phi (3 floats) + link (u32) + layer (u8) = 17 B, three faces a cell | | **~51** | solids (solids.cpp `solid_field_bytes`) |
| regions: label u32, flag u8, u32 | | 9 | fluid-region labels |
| CG vectors | | ~18 (*assumed*: `pcg_bytes` not itemised here) | |
| multigrid face weights | | ~14 (*derived*) | |
| render volume, staging, the rest | | ~20 to 30 (the remainder to 221/234.6) | |

**What can go, in order of size:**

1. **The solid field, ~51 B + regions 9 B + weights 14 B = ~75 B**: it exists for every cell, but solids touch a thin shell (the FPV frame and motors are a few percent of the box). **Make it sparse**: theta, wall, phi, link only inside a band 2 dx round the surfaces (a tile list, 4^3 or 8^3 cells a tile, with a per-tile index), regions labelled once per run and held coarse. Target: under 4 B a cell on average. **Saves ~70 B.** This is the biggest single saving and the one with no change to the numbers (the band is exactly the samples `build_solids` already visits: "every sample in the union of those boxes is sampled").
2. **Advection scratch, `hat`/`lo`/`hi` (36 B) and `next` (16 B)**: `lo` and `hi` are the limiter's bounds from the stencil's own neighbours: **recompute them in the kernel** instead of storing (the extra reads are cached; the saving is 6 floats). Fuse the passes so `next` is written in place by tiles. **Saves ~25 to 35 B.**
3. **Aliasing the pressure-solve scratch with the advection scratch** (they are never live together): the viscous right side, u/v right sides and du/dv/dw share one buffer set. **Saves ~15 to 20 B.**
4. **fp16 storage for u, v, w, p**: **not first.** Core's count: it saves 8 of the 221 B at most (the scratch and multigrid levels are the big items, and fp32 arithmetic with fp16 storage adds a CPU/GPU conversion contract). Left for later, behind a flag.
5. **Out-of-core**: §8.

**Target about 85 B a cell** (dense solver ~60 to 65 B, sparse solid ~4 B, render and staging ~20 B): **capacity 12.6 M cells a GiB against 4.6 to 4.9 now, so about 88 M cells on 7.0 GiB against 32 to 34 M.** *Derived*: the target is a plan, to be measured at the end of Phase 0 (an allocation probe per box size, which I am adding anyway).

## 2. Local refinement: nested patches

**Design.** Levels L0 (the 1.25 mm film box, as today), L1 0.625 mm, L2 0.3125 mm, and, for the near-blade band only, L3 0.156 mm (a later step). **Refinement ratio 2:1 in space; in time the finest level takes more steps than 2:1 gives it.** The tip moves 62 m/s: at L2 (0.3125 mm) the fluid CFL-1 step is 5 us and the solid-motion limit of half a cell is 2.5 us, so L2 steps at **3.125 us, 8 substeps to the coarse 25 us** (the tip then moves 0.62 cell a substep, a little over the half-cell target: flagged in §10); L1 (slower air, the wake leaving) takes 12.5 us, 2 substeps.

**Where the patches go (derived, not adaptive).** The patch set is a **pure function of the posed rotors**: for each rotor a cylinder round its disc (radius R + 4 mm, ±6 mm) at L2, a buffer cylinder round it at L1 (R + 12 mm, ±14 mm), both axis-aligned tiles (8^3). No regrid from the flow, so no stored decision and no flow-dependent traversal (Core's determinism point).

**Cell counts with refinement (derived, four rotors):**

| level | cell | per rotor | four rotors |
|---|---|---|---|
| L0 box | 1.25 mm | | 17.0 M (today's box) |
| L1 buffer | 0.625 mm | 2.05 M | 8.2 M |
| L2 disc | 0.3125 mm | 5.6 M | 22.5 M |
| L3 blade band (±1.5 mm round each blade's planform) | 0.156 mm | | ~10.8 M for 12 blades |

**L0+L1+L2 = 47.7 M cells for the whole FPV film at 0.3125 mm; at ~85 B that is 3.9 GiB: it fits.** At today's 221 B it is 10.5 GiB: it does not. **The memory diet is therefore a hard prerequisite, not an optimisation.** Without refinement the same 0.3125 mm over the film box would be 17 M x 64 = **1.1 billion cells**.

**Coupling across a patch face and its conservation test.** Fine boundary values come from the coarse level (space-time interpolation of u, v, w from the parent's two steps); the coarse level receives the fine level's fluxes by **reflux**: the coarse face flux is replaced by the sum of the fine faces' fluxes it covers, so mass is conserved to rounding across the face. Pressure: the fine level is solved with a Dirichlet condition from the interpolated coarse pressure, then the coarse level is corrected with the fine flux (a two-level fast adaptive composite scheme, one correction a step; the second order of its convergence to be measured, not claimed). **Tests (RED first):** (1) a uniform stream through a patch is exactly preserved (bit-for-bit zero change); (2) a closed box with a patch and a source: the sum of the coarse divergence and of the fine fluxes it covers vanishes to 1e-6 of the throughput; (3) a vortex ring crosses the patch face: its circulation and impulse before and after within a stated band, and the same ring in a uniform fine grid is the reference; (4) the patch's result with the patch the whole box equals the single-grid solver to the bit.

## 3. Moving solids: a cut-cell blade in a static fine patch

**Choice: an immersed (cut-cell) solid with its wall velocity in the pressure solve, on a static patch. Not a rotating patch.**

Why:
- A **rotating patch** (sliding interface, or a rotating frame) needs a non-conforming interface between a rotating mesh and a static one (interpolation that does not conserve), or Coriolis and centrifugal terms in a non-inertial frame and a rotating-to-static coupling. Both are new physics in a solver whose determinism rules are strict, and both give a worse coupling at exactly the face where the wake leaves the patch.
- The engine **already has the cut-cell machinery**: the solid field's `theta` (the face's open fraction, Batty, Bertails and Bridson 2007: `clamp(0.5 + phi/dx, 0, 1)`, closed under the 0.05 aperture floor) and `wall` (the owning part's rigid velocity at the sample), the Dirichlet viscous links, and the aperture-weighted multigrid (`set_weights`). What is missing is that all of that is built **once at start** and static; it has to be **rebuilt each step for the blade**, on the device, for the patch only.
- A moving solid in a fixed grid has the known problem of **fresh cells** (a cell that was solid last step, fluid now): its velocity is filled by extrapolation along the solid's normal from the fluid side, plus the solid's velocity; the step is limited to **about half a cell of solid motion** at the finest level (a 0.3125 mm level at 62 m/s: 2.5 us; the plan uses 3.125 us, §2; **this is what makes the film's resolved phase expensive**, §9).

**What the engine needs (all new):** (1) a **blade rasteriser** on the device: the blade's geometry (chord, twist, a thickness law: **assumed**, UIUC gives the chord and twist only) turned by the rotor's azimuth, into `theta`, `phi`, `wall` and the solid links for the cells of the patch (a sparse tile update); (2) the multigrid's face weights and the CG operator rebuilt for the patch's moving band; (3) the fresh-cell fill; (4) the CPU reference and the device twin, bit-compatible under the same rules as today; (5) the pressure readout on the blade surface (§7).

**Tests (RED first):** a plate moving through a closed box (the piston): the fluid's flux through a plane equals the plate's displacement rate to 1e-3; a plate turning in still air in a closed box: the air's angular momentum equals the plate's, to 1e-3; a sphere carried at constant speed through a periodic box: the drag against the known value at its Reynolds number within a stated band.

## 4. Symmetry savings, only where honest

- **A periodic single blade passage** (a 120-degree sector for a three-blade rotor, rotating frame, periodic faces): a 3x saving for a **hover check only**, never the film. But the engine's periodic boundary is an axis-aligned translation; a rotational periodicity needs a new boundary type (a remap of faces across a 120-degree seam). **Not worth building before the validation shows it is needed:** it would save about 2 of the 6 to 10 GPU hours of the validation runs, for ~8 days of work.
- **One rotor, not four** (an isolated rotor in its own box): a 4x saving for the validation and the first stills, honest because blade pressure at hover is set by the rotor's own flow, with the neighbours' inflow interference small but not zero; **labelled "one rotor, isolated" on screen.**

## 5. Cell size against what is resolved (derived; blade geometry from the library prop, thickness assumed)

The prop's mid-span chord is about 12 mm (c/R 0.195, R 63.5 mm), 10.6 mm at 0.75 R, 7.5 mm at 0.9 R; thickness **assumed** 1.0 to 1.5 mm; leading-edge radius **assumed** 0.1 to 0.2 mm. Tip speed 62 m/s at 9,298 rpm; **chord Reynolds number about 2.7e4 (0.5 R) to 3.5e4 (0.75 R)** (*derived*: U c / 1.5e-5). A laminar boundary layer at the trailing edge, 5 c / sqrt(Re): about **0.3 to 0.4 mm**. At these Reynolds numbers a laminar separation bubble is the usual condition; its height is a fraction of that.

| cell | chord in cells | thickness in cells | LE radius | boundary layer (0.3 mm) | what it resolves | cells for 4 rotors, with refinement |
|---|---|---|---|---|---|---|
| 0.625 mm | 19 | 1.6 to 2.4 | 0.2 to 0.3 cell | 0.5 cell | blade as a thin body; **net loading and its spanwise shape**; the pressure and suction side as two sides | L0 + L1(coarser) ~ 20 M |
| **0.3125 mm** | 38 | 3.2 to 4.8 | 0.3 to 0.6 cell | 1 cell | the blade as a **solid, not a sheet**; **loading and a smoothed suction peak** (smeared over ~2 to 3 cells = 5 to 8% of chord); the tip vortex's formation but not its core | **47.7 M** (L0+L1+L2) |
| 0.156 mm | 77 | 6.4 to 9.6 | 0.6 to 1.3 cells | 2 cells | **the leading-edge peak, a coarsely resolved boundary layer**, a laminar bubble only if it is taller than ~0.3 mm | + ~11 M for the blade band (L3), ~59 M |
| 0.100 mm | 120 | 10 to 15 | 1 to 2 cells | 3 cells | a boundary layer with 3 cells across it; separation marginal | **687 M for the disc at this size**; with a blade band only, ~45 M (with moving-patch tracking to build) |
| 0.03 mm | | | | 10 cells | a resolved BL (DNS-like) | not feasible on this card in any form |

**So the honest ladder:** 0.3125 mm is "blade loading from a solved flow"; 0.156 mm adds "the suction peak, to the extent the grid can hold it"; a viscous surface (skin friction, bubbles) is not on this card. The captions say which.

## 6. First payoff: the O158 film goes deeper

**Cameron's words (O185): "0.25 m now, deeper later."** Once Phase 0's sparse solid field lands, the same film box can go deeper at no change to the film: at ~85 B a cell the card holds ~88 M cells, which is the FPV box (252 x ny x 284) with **ny up to ~1,230 cells = 1.54 m**, against 437 (+0.25 m) in the first run. The wake front is ~0.8 to 1.3 m below the hub by the last shown step, so the whole wake fits. Time is not free: ~2 s a step at 88 M cells at today's 22.8 ns a cell (9,400 steps x 2.0 s = 5.2 h, over the 4 h day), so it also needs the traffic saving of items 2 and 3 above (less memory moved a step: **unmeasured**, to be measured at the end of Phase 0) or a two-day split. **This is the plan's first visible result, at about day 8.**

## 7. Validation (RED first, a stated band)

1. **Unit tests:** the moving plate and the periodic sphere (§3); the patch's conservation tests (§2).
2. **One resolved rotor against UIUC's measured static point.** The Micro Invent 5x4 3-blade has measured static C_T and C_P from 1,596 to 6,396 rpm (we hold the file, SHA recorded in the fit record): **at 6,057 rpm C_T 0.1815, C_P 0.1189** (*measured*, interpolated), with the measured scatter +-2.2% (C_T) and +-5.2% (C_P). **Band for the resolved rotor (0.3125 mm), a prediction, not a promise:** C_T within +-10% and C_P within +-15% of the measurement, at 4,000 and 6,057 rpm. **If it falls outside the band it is a result, reported as one**, with the first suspect named (the assumed thickness, the unresolved boundary layer, the box). The resolved rotor has **no fitted polar**, so a match is a real independent check of the line model's fit, and it gives a **prediction at 9,298 rpm**, the film's speed, where no data exist (UIUC stops at 6,396).
3. **The line against the resolved rotor, same box:** thrust and the spanwise loading, side by side.
4. **Then the four-rotor FPV film** with the resolved phase (§9), blade pressure painted from the fine patch.
5. **The thin-airfoil reconstruction** (a blade-element pressure shape laid over the line's loads, ~2 to 3 days, CPU) stays worth having **as a cross-check, built after the first resolved still exists** (to compare the chordwise shape of the solved loading with theory, and to tell a solver artefact from a real feature). It is **not** a substitute and is not started.

## 8. Out-of-core and streaming: where it honestly sits

Holding the fields in host RAM and streaming tiles to the card over PCIe (about 12 GB/s in practice on this machine class, *assumed*) works for a pass that reads each cell once (advection). The **pressure solve reads the whole field 8 to 12 times a step** (CG iterations, multigrid sweeps), so the traffic is 10x the single pass: for a 100 M-cell domain at ~85 B (8.5 GB) a step moves ~85 to 170 GB, **7 to 14 s of PCIe a step against ~2 s of compute**: **5 to 10 times slower, per step, for the whole run.** An honest place for it: **only the far-field coarse level L0 of a deep film** (the wake region where the flow is slow and smooth), updated with a longer time step than the fine levels, and the **frame output** (already streamed). **Not recommended** for the resolved blades, whose cost is in the patch, which must stay on the card.

## 9. Order, days, GPU hours

**Order against O158 and O159.** O158 (the line film, +0.25 m, floor 12, trimmed) and O159 (the ducted/bare cinewhoop) run on the line model **first**; fix 2 (the aperture-aware line: measured negligible, 2026-11-22, so a guard test instead of a kernel change), fix 3 (the propeller's speed trend) and fix 1 (the trim) come before either, as ordered. The resolved-blade work runs **alongside in code** and uses the GPU only on its own days after the O158 solve: no change to the O158/O159 schedule.

| Phase | Physics days | GPU h | what it shows on screen (Rendering's words) |
|---|---|---|---|
| 0. memory diet: sparse solid field, recomputed limiter bounds, aliased scratch, allocation probe; CPU/GPU twins, pins | 7 to 9 | 3 to 5 | the **deeper O158 wake** (first payoff) |
| 1. nested patches: levels, interpolation, reflux, subcycling, patch blob, CPU+GPU | 15 to 20 | 15 to 25 | a vortex ring crossing a patch face, seamless; the patch outlined in a still |
| 2. moving cut-cell blade: rasteriser, weights, fresh-cell fill, tests, CPU+GPU | 10 to 13 | 15 to 25 | **P1: blade pressure resolved on the surface of one rotor, beside the line's colour for the same frame** |
| 3. one resolved rotor, validation against UIUC, line comparison | 6 to 8 | 15 to 25 | P2: slow-motion clip of one rotor, suction building along each blade; P4 figures |
| 4. four rotors with patches; the hybrid film (line spin-up, then resolved phase); film run | 6 to 8 | 8 to 15 | **P3: the FPV pressure film with blades painted from the fine solve** |
| **Physics total** | **~45 to 58** | **~55 to 95** | |
| Core (layout, fp16 and packed cell types, blob v2, canonical order, pins and re-records) | 8 to 11 (11 to 15 if the hierarchy lives inside Simulation's state: not proposed) | | |
| Rendering (multi-level cache and reader 3, composite sampler and slice 2, clipped multi-layer march and seam blend 7, per-item surface colouring 3, stills and clips 3) | ~18 | | the on-screen steps above |

**The hybrid film.** The wake needs ~0.2 s to develop (the film shows 0.2 to 0.235 s) and the resolved phase costs ~4.9 s a 25 us coarse step with four rotors at 0.3125 mm (*derived*: L0 17.0 M + 2 x L1 8.2 M + 8 x L2 22.5 M = 213 M cell-updates a coarse step at 22.8 ns), **~21 minutes a revolution** (258 coarse steps a turn), so a whole film at full resolution is ~12.8 h and the 1.5 turns shown ~32 min. **Therefore: run the line model on the coarse box to a developed wake (as the O158 film does), then switch the patches on for the last 3 to 4 revolutions** (~1.0 to 1.4 h); the blade pressure converges within about 2 to 3 turns of the patches switching on (*assumed*, to be measured in Phase 3).

**First milestone that shows Cameron something real, with two stops:** (a) **~day 8**: the deeper O158 wake from the memory diet; (b) **~day 35 (Phases 0 to 3)**: **one resolved rotor, in a nested patch, blade pressure on screen, thrust and power compared to UIUC's measured point with the stated band.** A first stop on the way, **~day 22** (Phases 0 and 1): a patch round a static plate with the conservation tests green and a vortex crossing its face.

## 10. Risks, named

- **Fresh-cell noise** from the moving solid (the classic immersed-boundary pressure spikes): mitigated by half-cell steps, extrapolated fills, and the moving-plate tests; if not enough, a smoothed `theta` ramp across two cells; a real risk to the 12-13 days of Phase 2.
- **Reflux/subcycling in a GPU solver with strict determinism**: the canonical order (Morton, patch then cell, no float atomics) is Core's rule; the cost is a few kernels per level.
- **The fine patch's time step** (3.1 us, 8 substeps a coarse step) makes the resolved phase about 13 times the line model's cost per coarse step (4.9 s against 0.39 s): handled by the hybrid film; the estimate is *derived*, not measured; the tip's 0.62 cell a substep is above the half-cell target, so the fresh-cell tests run at the tip speed first.
- **The assumed blade thickness** (1.0 to 1.5 mm) sets the surface pressure's suction side: stated in every caption; the sensitivity is a Phase 3 run (+-0.3 mm).
- **Rendering's clipped multi-layer march** (7 days, GL shader, the whole `-L gpu` gate and a CPU-against-GL band): Rendering names it the main risk.
- **Core's estimate is +-40%** (read from headers only), mine +-30% on Phases 1 and 2 (no experience of a nested solver in this engine yet).

## 11. What each realm owes, and what I am asking

- **Core:** blob v2 (a patch table of integer boxes and per-patch field arrays, canonical order, a digest), the packed-cell and fp16 types behind a flag, the layout and pin rows, the re-record of every pin a shared struct edit touches (new structs only, none edited); Core's days above.
- **Rendering:** the multi-level cache and reader, composite slice, clipped march, per-item surface colouring. **Their questions, answered:** the patch data's file layout is Core's blob v2: per level an integer origin in its parent's cells, an integer size, the cell size (1.25, 0.625, 0.3125, 0.156 mm), the channels (u, v, w, p as fp32, and the render volume's q-hat and pressure as the film writes them), one record per level per frame; the cell sizes are the four above; **the pressure the blade surface carries is the cell-centred p of the cell touching the blade, extrapolated to the wall by the one-sided gradient (first order); on a blade thin enough to be a sheet at a level, it is the jump across the sheet. The caption says "pressure in the air against the blade", not "surface Cp".**
- **The lead:** the order against O158/O159 as above; **a decision, not now, on whether Phases 2 to 4 are worth ~50 working days**; Phase 0 stands on its own (a deeper wake) and I propose to do it whatever the answer.
